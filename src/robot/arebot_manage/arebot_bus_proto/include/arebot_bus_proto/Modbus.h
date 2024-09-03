/**
 * Copyright (c) 2021 Chuanhong Guo <gch981213@gmail.com>
 */
 /* changed for arebot version in 2022.9*/
#ifndef BUS_PROTO_MODBUS_H
#define BUS_PROTO_MODBUS_H

#include <ros/ros.h>
#include <cstdint>
#include <vector>

namespace AREBot_ROS {
    namespace Proto {
        class Modbus {
        private:
            ros::ServiceClient service_client;
            uint8_t bus_id;

            static uint16_t modbus_rtu_crc(const uint8_t *buf, uint8_t len);

            template<class Service>
            bool call_service(Service &service) {
                if (!service_client.exists()) {
                    ROS_WARN("Bus service unavailable. Waiting...");
                    service_client.waitForExistence();
                    ROS_INFO("Bus service ready.");
                }
                if (!service_client.call(service)) {
                    ROS_WARN("service call failed.");
                    return false;
                }
                return true;
            }

            bool read_registers(uint8_t addr, uint8_t func_code, uint16_t start_addr, uint16_t len,
                                std::vector<uint16_t> &ret_buf);

        public:
            Modbus(ros::NodeHandle &private_nh, uint8_t _bus_id);

            bool read_holding_registers(uint8_t addr, uint16_t start, uint16_t len, std::vector<uint16_t> &ret_buf) {
                return read_registers(addr, 0x03, start, len, ret_buf);
            }

            bool read_input_registers(uint8_t addr, uint16_t start, uint16_t len, std::vector<uint16_t> &ret_buf) {
                return read_registers(addr, 0x04, start, len, ret_buf);
            }

            bool preset_register(uint8_t addr, uint16_t reg_addr, uint16_t val);

            bool preset_registers(uint8_t addr, uint16_t reg_addr, const std::vector<uint16_t> &val);
        };
    }
}

#endif //BUS_PROTO_MODBUS_H