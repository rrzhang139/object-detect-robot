/**
 * Copyright (c) 2021 Chuanhong Guo <gch981213@gmail.com>
 */

#ifndef SRC_ULTRASONICNODE_H
#define SRC_ULTRASONICNODE_H

#include <memory>
#include <string>
#include <list>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <arebot_control/BusOp.h>

namespace AREBot_ROS {

    class UltrasonicNode : public nodelet::Nodelet {
    public:
        virtual void onInit();

    private:
        int sonar_bus;
        ros::Timer poll_timer;
        ros::ServiceClient service_client;
        int range_min, range_max;
        ros::Publisher pub;
        std::string frame;

        void poll(const ros::TimerEvent &e);
    };
}

#endif //SRC_ULTRASONICNODE_H
