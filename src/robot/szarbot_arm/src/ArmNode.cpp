/**
 * Copyright (c) 2021 Chuanhong Guo <gch981213@gmail.com>
 */

#include <chrono>
#include <thread>
#include <vector>
#include <cstdio>
#include <szarbot_arm/ArmNode.h>
#include <pluginlib/class_list_macros.h>
#include <arebot_control/BusOp.h>

#define CMD_SUCTION_ON 0x25
#define CMD_SUCTION_OFF 0x26
#define CMD_POS 0x27

namespace SZARBot_ROS {
    void ArmNode::onInit() {
        ros::NodeHandle private_nh = getPrivateNodeHandle();
        private_nh.param<int>("arm_bus", bus_id, 2);
        pos_sub = private_nh.subscribe("position", 1, &ArmNode::position_cb, this);
        suc_sub = private_nh.subscribe("suction_cup", 1, &ArmNode::suction_cup_cb, this);
        std::string bus_service_name;
        private_nh.param<std::string>("bus_service_name", bus_service_name, "bus_op");
        service_client = private_nh.serviceClient<arebot_control::BusOp>(bus_service_name);
    }

    void ArmNode::position_cb(const geometry_msgs::Point::ConstPtr &msg) {
        arebot_control::BusOp bus_op;
        uint16_t x = msg->x * 10, y = msg->y * 10, z = msg->z * 10;
        bus_op.request.bus_id = bus_id;
        bus_op.request.data.reserve(14);
        bus_op.request.data = {0xfd, 0xfd, 0xfc, 0x06, 0x07, 0x00, CMD_POS};
        bus_op.request.data.resize(14);
        bus_op.request.data[7] = x & 0xff;
        bus_op.request.data[8] = x >> 8;
        bus_op.request.data[9] = y & 0xff;
        bus_op.request.data[10] = y >> 8;
        bus_op.request.data[11] = z & 0xff;
        bus_op.request.data[12] = z >> 8;
        bus_op.request.data[13] = 0xff;
        bus_op.request.recv_len = 0;
        service_client.call(bus_op);
    }

    void ArmNode::suction_cup_cb(const std_msgs::Bool::ConstPtr &msg) {
        arebot_control::BusOp bus_op;
        bus_op.request.bus_id = bus_id;
        bus_op.request.data = {0xfd, 0xfd, 0xfc, 0x06, 0x01, 0x00, CMD_SUCTION_OFF, 0xff};
        if (msg->data)
            bus_op.request.data[6] = CMD_SUCTION_ON;
        bus_op.request.recv_len = 0;
        service_client.call(bus_op);
    }
}

PLUGINLIB_EXPORT_CLASS(SZARBot_ROS::ArmNode, nodelet::Nodelet)
