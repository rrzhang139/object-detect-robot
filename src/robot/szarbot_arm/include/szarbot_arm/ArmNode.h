/**
 * Copyright (c) 2021 Chuanhong Guo <gch981213@gmail.com>
 */

#ifndef SRC_ARMNODE_H
#define SRC_ARMNODE_H

#include <memory>
#include <string>
#include <list>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>

namespace SZARBot_ROS {
    class ArmNode : public nodelet::Nodelet {
    public:
        virtual void onInit();

    private:
        int bus_id;
        ros::Subscriber pos_sub, suc_sub;
        ros::ServiceClient service_client;

        void position_cb(const geometry_msgs::Point::ConstPtr &msg);

        void suction_cup_cb(const std_msgs::Bool::ConstPtr &msg);
    };
}

#endif //SRC_ARMNODE_H
