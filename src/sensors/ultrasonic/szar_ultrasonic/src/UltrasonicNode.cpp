/**
 * Copyright (c) 2021 Chuanhong Guo <gch981213@gmail.com>
 */

#include <chrono>
#include <thread>
#include <vector>
#include <cstdio>
#include <szar_ultrasonic/UltrasonicNode.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Range.h>
extern "C" {
    #include "dynamixel_rd.h"
}

namespace AREBot_ROS {
    void UltrasonicNode::onInit() {
        ros::NodeHandle private_nh = getPrivateNodeHandle();
        double poll_freq;
        std::string bus_service_name;

        private_nh.param<int>("sonar_bus", sonar_bus, 1);
        private_nh.param<double>("poll_frequency", poll_freq, 5.0);
        private_nh.param<int>("range_min", range_min, 5);
        private_nh.param<int>("range_max", range_max, 1900);
        std::string sensor_pub_topic;
        private_nh.param<std::string>("topic", sensor_pub_topic, "range");
        pub = private_nh.advertise<sensor_msgs::Range>(sensor_pub_topic, 1);

        private_nh.param<std::string>("bus_service_name", bus_service_name, "bus_op");
        service_client = private_nh.serviceClient<arebot_control::BusOp>(bus_service_name);

        private_nh.param<std::string>("frame", frame, "sonar");

        poll_timer = private_nh.createTimer(ros::Duration(1 / poll_freq), &UltrasonicNode::poll, this);
    }

    void UltrasonicNode::poll(const ros::TimerEvent &e) {
        arebot_control::BusOp bus_op;
        bus_op.request.bus_id = sonar_bus;
        bus_op.request.data.resize(8);
        long Value[2] = {0};
        Value[0] = 0;
        unsigned char ReturnData[30];
        unsigned char Send_Length = Dynamixel_Master_Send(Dynamixel_ID_Ul, 0x02, 0x02, Value, 4, bus_op.request.data.data());
        bus_op.request.recv_len = 11;

        if (!service_client.exists())
        {
            ROS_WARN("Bus service unavailable. Waiting...");
            service_client.waitForExistence();
            ROS_INFO("Bus service ready.");
        }
        if (!service_client.call(bus_op))
        {
            ROS_WARN("service call failed.");
            return;
        }

        unsigned int uRXflag = Dynamixel_Master_Receive(bus_op.response.data.data(), bus_op.response.data.size(), ReturnData); // stm32->AR
        if (uRXflag != Dynamixel_State_Success)
        {
            ROS_WARN("incorrect data?");
            for(auto i:bus_op.response.data)
                ROS_WARN("%02x ", i);
            return;
        }
        float range = Dynamixel_Value_Forward(ReturnData, 3, 3 + 3);

        sensor_msgs::Range::Ptr msg(new sensor_msgs::Range);
        msg->header.frame_id = frame;
        msg->header.stamp = ros::Time::now();
        msg->radiation_type = sensor_msgs::Range::ULTRASOUND;
        msg->field_of_view = M_PI / 6; // XXX: This value needs to be checked!
        msg->min_range = range_min / 1000.0;
        msg->max_range = range_max / 1000.0;
        if (range < range_min)
            msg->range = msg->min_range;
        else if (range > range_max)
            msg->range = msg->max_range;
        else
            msg->range = range / 1000.0;
        pub.publish(msg);
    }
}

PLUGINLIB_EXPORT_CLASS(AREBot_ROS::UltrasonicNode, nodelet::Nodelet)
