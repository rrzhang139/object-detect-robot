/**
 * Copyright (c) 2021 Chuanhong Guo <gch981213@gmail.com>
 */
/*changed for arebot in 2022.9*/
#ifndef BOTCONTROL_H
#define BOTCONTROL_H

#include <BotControl.h>
#include <arebot_control/BusOp.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <memory>
#include <future>
#include <vector>
#include <exception>

namespace AREBot_ROS {
    class BotControlNodeCB : public AREBot::BotControlCallbacks {
    private:
        ros::Publisher imu_pub;
        ros::Publisher bat_pub;
        ros::Publisher bumper_pub;
        double angular_velocity_covariance, pitch_roll_covariance, yaw_covariance, linear_acceleration_covariance;
        std::string imu_frame_id;
        std::vector<std::promise<std::vector<char>>> bus_promises;
        std::vector<std::mutex> bus_mutexes;
    public:
        BotControlNodeCB(ros::NodeHandle &private_nh, int num_buses = 4);

        virtual void imu_data(const AREBot::IMUData &data);

        virtual void battery_voltage(float voltage);

        virtual void bumper_status(uint32_t status);

        virtual void bus_response(uint8_t bus_id, const std::vector<char> &buf);

        void lock_bus(int bus_id) {
            if (bus_id < bus_mutexes.size())
                bus_mutexes[bus_id].lock();
        }

        void unlock_bus(int bus_id) {
            if (bus_id < bus_mutexes.size())
                bus_mutexes[bus_id].unlock();
        }

        std::future<std::vector<char>> prepare_recv(int bus_id) {
            if (bus_id >= bus_promises.size())
                throw std::out_of_range("invalid bus id.");
            // create a new promise and destroy the old one.
            bus_promises[bus_id] = std::promise<std::vector<char>>();
            return bus_promises[bus_id].get_future();
        }

        virtual ~BotControlNodeCB() {}
    };

    class BotControlNode : public nodelet::Nodelet {
    private:
        ros::ServiceServer bus_op_srv;
        bool terminate_thread;
        std::thread board_polling_thread;
        // We need to construct this later with a proper serial port.
        std::unique_ptr<AREBot::BotControl> board;
        std::shared_ptr<BotControlNodeCB> board_callbacks;

        void board_comm_worker();
        void bus_setup();

    public:
        virtual void onInit();

        bool bus_op(arebot_control::BusOpRequest &req, arebot_control::BusOpResponse &resp);

        virtual ~BotControlNode();
    };

}

#endif //BOTCONTROL_H
