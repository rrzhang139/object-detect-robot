/**
 * Copyright (c) 2021 Chuanhong Guo <gch981213@gmail.com>
 */

#include <arebot_base/BaseHW.h>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>

#define PARAM_KP_LEVEL 0x1
#define PARAM_KI_LEVEL 0x2
#define PARAM_KD_LEVEL 0x4
#define REG_PROTOREV 0x05
#define REG_KP 0x10
#define REG_KI 0x11
#define REG_KD 0x12
#define REG_PID_INTERVAL 0x13

namespace AREBot_ROS {
    BaseHW::BaseHW(ros::NodeHandle &private_nh, uint8_t bus_id) : modbus(private_nh, bus_id), reconfigure_server(private_nh) {
        int tmp;
        private_nh.param<int>("encoder_count_per_cycle", tmp, 1580);
        encoder_count_per_cycle = (int16_t) tmp;
        private_nh.param<int>("motor_board_addr", tmp, 1);
        addr = (uint8_t) tmp;
        std::vector<uint16_t> reg;
        if (!modbus.read_input_registers(addr, REG_PROTOREV, 1, reg))
            throw std::runtime_error("failed to read motor protocol revision.");
        if (reg[0] < 2)
            throw std::runtime_error("Motor protocol revision too low. Please upgrade the motor control board firmware.");
        private_nh.param<int>("pid_interval", tmp, 50);
        if (!modbus.preset_register(addr, REG_PID_INTERVAL, (uint16_t) tmp))
            throw std::runtime_error("Failed to set PID interval.");
        motor_vel_interval = tmp / 1000.0;
        for (int i = 0; i < num_wheels; i++) {
            std::string joint_name = std::string("wheel_") + std::to_string(i) + "_joint";
            private_nh.param<bool>(std::string("wheel_") + std::to_string(i) + "_reverse", reverse_speed[i], false);
            position[i] = 0;
            prev_position[i] = 0;
            velocity[i] = 0;
            effort[i] = 0;
            target_velocity[i] = 0;
            hardware_interface::JointStateHandle state_handle(joint_name, &position[i], &velocity[i], &effort[i]);
            joint_state_interface.registerHandle(state_handle);
            hardware_interface::JointHandle vel_handle(joint_state_interface.getHandle(joint_name),
                                                       &target_velocity[i]);
            joint_vel_interface.registerHandle(vel_handle);
        }
        registerInterface(&joint_state_interface);
        registerInterface(&joint_vel_interface);
        reconfigure_server.setCallback(boost::bind(&BaseHW::motor_pid_reconfigure, this, _1, _2));
    }

    bool BaseHW::motor_read() {
        std::vector<uint16_t> regs;
        if (!modbus.read_input_registers(addr, 0, 2 * num_wheels, regs))
            return false;
        for (int i = 0; i < num_wheels; i++) {
            int16_t pos_diff = regs[i * 2] - prev_position[i];
            if(reverse_speed[i])
                pos_diff = -pos_diff;
            position[i] += (double) pos_diff / encoder_count_per_cycle * 2 * M_PI;
            prev_position[i] = regs[i * 2];
            velocity[i] = regs[i * 2 + 1] / encoder_count_per_cycle * 2 * M_PI / motor_vel_interval;
            if(reverse_speed[i])
                velocity[i] = -velocity[i];
        }
        return true;
    }

    bool BaseHW::motor_write() {
        std::vector<uint16_t> regs(2);
        for (int i = 0; i < num_wheels; i++) {
            int16_t tgt_vel = (int16_t) (target_velocity[i] / (2 * M_PI) * motor_vel_interval *
                                         encoder_count_per_cycle);
            if(reverse_speed[i])
                tgt_vel = -tgt_vel;
            regs[i] = tgt_vel;
        }
        return modbus.preset_registers(addr, 0, regs);
    }

    void BaseHW::motor_pid_reconfigure(arebot_base::MotorPIDConfig &config, uint32_t level) {
        if (level & PARAM_KP_LEVEL)
            if (!modbus.preset_register(addr, REG_KP, (uint16_t) (config.kp * 1000)))
                ROS_WARN("Failed to set Kp.");

        if (level & PARAM_KI_LEVEL)
            if (!modbus.preset_register(addr, REG_KI, (uint16_t) (config.ki * 1000)))
                ROS_WARN("Failed to set Ki.");

        if (level & PARAM_KD_LEVEL)
            if (!modbus.preset_register(addr, REG_KD, (uint16_t) (config.kd * 1000)))
                ROS_WARN("Failed to set Kd.");

        std::vector<uint16_t> regs;
        if (!modbus.read_holding_registers(addr, REG_KP, 3, regs)) {
            ROS_WARN("Failed to read back configured PID values.");
            return;
        }
        ROS_INFO("Updated motor PID: %.4lf, %.4lf, %.4lf", regs[0] / 1000.0, regs[1] / 1000.0, regs[2] / 1000.0);
    }
}