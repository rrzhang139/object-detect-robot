/**
 * Copyright (c) 2021 Chuanhong Guo <gch981213@gmail.com>
 */

#include <pluginlib/class_list_macros.h>
#include <arebot_safety_controller/SafetyControllerNode.h>

namespace AREBot_ROS {
    void SafetyControllerNode::onInit() {
        ros::NodeHandle &private_nh = getPrivateNodeHandle();
        ros::NodeHandle &nh = getNodeHandle();
        std::string bumper_topic;
        private_nh.param<std::string>("bumper_topic", bumper_topic, "/arebot_control/bumper");
        private_nh.param<int>("bumper_left", bumper_left, 2);
        private_nh.param<int>("bumper_mid", bumper_mid, 1);
        private_nh.param<int>("bumper_right", bumper_right, 0);
        vel_pub = private_nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
        bumper_sub = nh.subscribe(bumper_topic, 3, &SafetyControllerNode::bumperCallback, this);
        pub_cnt = 0;
        pub_timer = private_nh.createTimer(ros::Duration(0.1), &SafetyControllerNode::publishVelCB, this);
    }

    void SafetyControllerNode::bumperCallback(const std_msgs::UInt32::ConstPtr &msg) {
        if (!msg->data)
            return;
        geometry_msgs::Twist::Ptr msg_ptr(new geometry_msgs::Twist);
        msg_ptr->linear.x = -0.1;
        msg_ptr->linear.y = 0.0;
        msg_ptr->linear.z = 0.0;
        msg_ptr->angular.x = 0.0;
        msg_ptr->angular.y = 0.0;
        if (msg->data & (1 << bumper_left))
            msg_ptr->angular.z = -0.4;
        else if (msg->data & (1 << bumper_right))
            msg_ptr->angular.z = 0.4;
        // nothing to change for the mid bumper
        vel = msg_ptr;
        // publish the speed 10 times to escape
        pub_cnt = 10;
        pub_timer.start();
    }

    void SafetyControllerNode::publishVelCB(const ros::TimerEvent &e) {
        if (pub_cnt == 0) {
            geometry_msgs::Twist::Ptr msg_ptr(new geometry_msgs::Twist);
            msg_ptr->linear.x = 0.0;
            msg_ptr->linear.y = 0.0;
            msg_ptr->linear.z = 0.0;
            msg_ptr->angular.x = 0.0;
            msg_ptr->angular.y = 0.0;
            msg_ptr->angular.z = 0.0;
            vel_pub.publish(msg_ptr);
            pub_timer.stop();
            return;
        }
        pub_cnt--;
        vel_pub.publish(vel);
    }
}

PLUGINLIB_EXPORT_CLASS(AREBot_ROS::SafetyControllerNode, nodelet::Nodelet);