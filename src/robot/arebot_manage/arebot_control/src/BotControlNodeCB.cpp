/**
 * Copyright (c) 2021 Chuanhong Guo <gch981213@gmail.com>
 */
/*changed for arebot in 2022.9*/
#include <arebot_control/BotControlNode.h>
#include <nodelet/nodelet.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <cmath>

#define ONE_G 9.80665

namespace AREBot_ROS {
    BotControlNodeCB::BotControlNodeCB(ros::NodeHandle &private_nh, int num_buses) : bus_promises(num_buses),
                                                                                     bus_mutexes(num_buses) {
        // IMU variance parameters
        // copied from: https://github.com/chrisspen/ros_mpu6050_node/blob/master/src/mpu6050_node.cpp
        double linear_acceleration_stdev_, angular_velocity_stdev_, yaw_stdev_, pitch_roll_stdev_;
        // NOISE PERFORMANCE: Power Spectral Density @10Hz, AFS_SEL=0 & ODR=1kHz 400 ug/√Hz (probably wrong)
        private_nh.param("linear_acceleration_stdev", linear_acceleration_stdev_, (400 / 1000000.0) * 9.807);
        // Total RMS Noise: DLPFCFG=2 (100Hz) 0.05 º/s-rms (probably lower (?) @ 42Hz)
        private_nh.param("angular_velocity_stdev", angular_velocity_stdev_, 0.05 * (M_PI / 180.0));
        // 1 degree for pitch and roll
        private_nh.param("pitch_roll_stdev", pitch_roll_stdev_, 1.0 * (M_PI / 180.0));
        // 5 degrees for yaw
        private_nh.param("yaw_stdev", yaw_stdev_, 5.0 * (M_PI / 180.0));
        angular_velocity_covariance = angular_velocity_stdev_ * angular_velocity_stdev_;
        linear_acceleration_covariance = linear_acceleration_stdev_ * linear_acceleration_stdev_;
        pitch_roll_covariance = pitch_roll_stdev_ * pitch_roll_stdev_;
        yaw_covariance = yaw_stdev_ * yaw_stdev_;

        private_nh.param<std::string>("imu_frame_id", imu_frame_id, "imu_link");

        bat_pub = private_nh.advertise<std_msgs::Float32>("battery_voltage", 5);
        bumper_pub = private_nh.advertise<std_msgs::UInt32>("bumper", 5);
        imu_pub = private_nh.advertise<sensor_msgs::Imu>("imu_data", 5);
    }

    void BotControlNodeCB::imu_data(const AREBot::IMUData &data) {
        sensor_msgs::ImuPtr msg(new sensor_msgs::Imu);
        msg->header.stamp = ros::Time::now();
        msg->header.frame_id = imu_frame_id;

        msg->orientation.w = data.orientation.w;
        msg->orientation.x = data.orientation.x;
        msg->orientation.y = data.orientation.y;
        msg->orientation.z = data.orientation.z;
        msg->angular_velocity.x = data.angular_velocity.x / 180 * M_PI;
        msg->angular_velocity.y = data.angular_velocity.y / 180 * M_PI;
        msg->angular_velocity.z = data.angular_velocity.z / 180 * M_PI;
        msg->linear_acceleration.x = data.linear_acceleration.x * ONE_G;
        msg->linear_acceleration.y = data.linear_acceleration.y * ONE_G;
        msg->linear_acceleration.z = data.linear_acceleration.z * ONE_G;

        msg->linear_acceleration_covariance[0] = linear_acceleration_covariance;
        msg->linear_acceleration_covariance[4] = linear_acceleration_covariance;
        msg->linear_acceleration_covariance[8] = linear_acceleration_covariance;
        msg->angular_velocity_covariance[0] = angular_velocity_covariance;
        msg->angular_velocity_covariance[4] = angular_velocity_covariance;
        msg->angular_velocity_covariance[8] = angular_velocity_covariance;
        msg->orientation_covariance[0] = pitch_roll_covariance;
        msg->orientation_covariance[4] = pitch_roll_covariance;
        msg->orientation_covariance[8] = yaw_covariance;

        imu_pub.publish(msg);
    }

    void BotControlNodeCB::battery_voltage(float voltage) {
        std_msgs::Float32Ptr msg(new std_msgs::Float32);
        msg->data = voltage;
        bat_pub.publish(msg);
    }

    void BotControlNodeCB::bumper_status(uint32_t status) {
        std_msgs::UInt32Ptr msg(new std_msgs::UInt32);
        msg->data = status;
        bumper_pub.publish(msg);
    }

    void BotControlNodeCB::bus_response(uint8_t bus_id, const std::vector<char> &buf) {
        if (bus_id < bus_promises.size()) {
            try {
                bus_promises[bus_id].set_value(buf);
            } catch (std::future_error &e) {
                ROS_WARN("Unexpected packet from bus %hhu.", bus_id);
            }
        }
    }
}