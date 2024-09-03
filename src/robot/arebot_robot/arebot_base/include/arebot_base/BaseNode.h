/**
 * Copyright (c) 2021 Chuanhong Guo <gch981213@gmail.com>
 */
/*changed for arebot in 2022.9*/
#ifndef SRC_BASENODE_H
#define SRC_BASENODE_H
#include <nodelet/nodelet.h>
#include <arebot_base/BaseHW.h>
#include <controller_manager/controller_manager.h>
#include <memory>
#include <thread>
namespace AREBot_ROS {
    class BaseNode : public nodelet::Nodelet {
    public:
        virtual void onInit();
        void motor_init();
        virtual ~BaseNode();

    private:
        std::unique_ptr<BaseHW> hw;
        std::unique_ptr<controller_manager::ControllerManager> cm;
        ros::Timer control_timer;
        std::thread init_thread;
        void control_loop(const ros::TimerEvent &e);
    };
}

#endif //SRC_BASENODE_H
