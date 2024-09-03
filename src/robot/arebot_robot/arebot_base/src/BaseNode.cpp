/**
 * Copyright (c) 2021 Chuanhong Guo <gch981213@gmail.com>
 */

#include <pluginlib/class_list_macros.h>
#include <arebot_base/BaseNode.h>

namespace AREBot_ROS {
    void BaseNode::onInit() {
        init_thread = std::thread(&BaseNode::motor_init, this);
    }

    BaseNode::~BaseNode() {
        init_thread.join();
    }

    void BaseNode::motor_init() {
        ros::NodeHandle private_nh = getPrivateNodeHandle();
        int tmp;
        double ctrl_freq;
        private_nh.param<int>("motor_bus_addr", tmp, 0);
        private_nh.param<double>("control_frequency", ctrl_freq, 15.0);
        hw = std::make_unique<BaseHW>(private_nh, tmp);
        cm = std::make_unique<controller_manager::ControllerManager>(hw.get());
        control_timer = private_nh.createTimer(ros::Duration(1 / ctrl_freq), &BaseNode::control_loop, this);
    }

    void BaseNode::control_loop(const ros::TimerEvent &e) {
        hw->motor_read();
        cm->update(e.current_real, e.current_real - e.last_real);
        hw->motor_write();
    }
}

PLUGINLIB_EXPORT_CLASS(AREBot_ROS::BaseNode, nodelet::Nodelet);