/**
 * Copyright (c) 2021 Chuanhong Guo <gch981213@gmail.com>
 */
/*changed for arebot in 2022.9*/
#ifndef SRC_BASEHW_H
#define SRC_BASEHW_H
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <arebot_bus_proto/Modbus.h>
#include <dynamic_reconfigure/server.h>
#include <arebot_base/MotorPIDConfig.h>

namespace AREBot_ROS {
    class BaseHW: public hardware_interface::RobotHW {
        constexpr static const int num_wheels = 2;
    public:
        BaseHW(ros::NodeHandle &private_nh, uint8_t bus_id = 0);
        bool motor_read();
        bool motor_write();
    private:
        double position[num_wheels];
        int16_t prev_position[num_wheels];
        double velocity[num_wheels];
        double effort[num_wheels];
        double target_velocity[num_wheels];
        bool reverse_speed[num_wheels];
        double motor_vel_interval;
        int16_t encoder_count_per_cycle;
        uint8_t addr;
        Proto::Modbus modbus;
        hardware_interface::JointStateInterface joint_state_interface;
        hardware_interface::VelocityJointInterface joint_vel_interface;
        dynamic_reconfigure::Server<arebot_base::MotorPIDConfig> reconfigure_server;
        void motor_pid_reconfigure(arebot_base::MotorPIDConfig &config, uint32_t level);
    };
}

#endif //SRC_BASEHW_H
