/**
 * Copyright (c) 2021 Chuanhong Guo <gch981213@gmail.com>
 */

#ifndef SRC_SAFETYCONTROLLERNODE_H
#define SRC_SAFETYCONTROLLERNODE_H
#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <geometry_msgs/Twist.h>
#include <nodelet/nodelet.h>

namespace AREBot_ROS {
    class SafetyControllerNode : public nodelet::Nodelet {
    private:
        ros::Publisher vel_pub;
        ros::Subscriber bumper_sub;
        ros::Timer pub_timer;
        geometry_msgs::TwistPtr vel;
        int bumper_left, bumper_mid, bumper_right;
        int pub_cnt;
        void bumperCallback(const std_msgs::UInt32::ConstPtr& msg);
        void publishVelCB(const ros::TimerEvent& e);
    public:
        virtual void onInit();
    };
}
#endif //SRC_SAFETYCONTROLLERNODE_H
