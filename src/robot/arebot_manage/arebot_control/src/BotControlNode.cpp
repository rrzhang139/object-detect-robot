/**
 * Copyright (c) 2021 Chuanhong Guo <gch981213@gmail.com>
 */
/*changed for arebot version in 2022.9*/
#include <arebot_control/BotControlNode.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <string>
#include <chrono>
#include <cstring>

namespace AREBot_ROS {
    BotControlNode::~BotControlNode() {
        terminate_thread = true;
        board_polling_thread.join();
    }

    void BotControlNode::bus_setup() {
        ros::NodeHandle private_nh = getPrivateNodeHandle();
        XmlRpc::XmlRpcValue bus_baudrates;
        if(!private_nh.getParam("bus_baudrates", bus_baudrates))
            return;
        for (int i = 0; i < bus_baudrates.size(); ++i) {
            int addr = static_cast<int>(bus_baudrates[i]["bus_id"]);
            int baud = static_cast<int>(bus_baudrates[i]["baudrate"]);
            int stop = static_cast<int>(bus_baudrates[i]["stop"]);
            std::string parity_str = static_cast<std::string>(bus_baudrates[i]["parity"]);
            board->bus_baudrate(addr, baud, parity_str[0], stop);
        }
    }

    void BotControlNode::onInit() {
        board_polling_thread = std::thread(&BotControlNode::board_comm_worker, this);
    }

    void BotControlNode::board_comm_worker() {
        ros::NodeHandle &private_nh = getPrivateNodeHandle();
        std::string bot_port;
        private_nh.param<std::string>("port", bot_port, "/dev/ttyACM0");
        board_callbacks = std::make_shared<BotControlNodeCB>(private_nh);
        board = std::make_unique<AREBot::BotControl>(*board_callbacks);
        board->open(bot_port.c_str(), 115200);
        bus_setup();
        board->imu_init(50, true);
        bus_op_srv = private_nh.advertiseService("bus_op", &BotControlNode::bus_op, this);
        terminate_thread = false;
        do {
            board->process_response();
        } while (!terminate_thread);
    }

    bool BotControlNode::bus_op(arebot_control::BusOpRequest &req, arebot_control::BusOpResponse &resp) {
        bool ret = false;
        board_callbacks->lock_bus(req.bus_id);
        try {
            if (req.recv_len) {
                auto recv_future = board_callbacks->prepare_recv(req.bus_id);
                board->bus_request(req.bus_id, req.data.size(), req.recv_len, req.data.data());
                if (recv_future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
                    // unable to assign vectors directly due to type mismatch.
                    auto resp_data = recv_future.get();
                    resp.data.resize(resp_data.size());
                    memcpy(resp.data.data(), resp_data.data(), resp_data.size());
                    ret = true;
                } else {
                    NODELET_WARN("bus op timeout");
                }
            } else {
                ret = !board->bus_request(req.bus_id, req.data.size(), req.recv_len, req.data.data());
            }
        } catch (std::out_of_range &e) {
            NODELET_WARN("invalid bus id");
        }
        board_callbacks->unlock_bus(req.bus_id);
        return ret;
    }
}

PLUGINLIB_EXPORT_CLASS(AREBot_ROS::BotControlNode, nodelet::Nodelet)