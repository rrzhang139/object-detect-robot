/**
 * Copyright (c) 2021 Chuanhong Guo <gch981213@gmail.com>
 */
/*changed for arebot version in 2022.9*/

#include <arebot_bus_proto/Modbus.h>
#include <arebot_control/BusOp.h>
#include <string>
#include <cstring>

AREBot_ROS::Proto::Modbus::Modbus(ros::NodeHandle &private_nh, uint8_t _bus_id) : bus_id(_bus_id) {
    std::string bus_service_name;
    private_nh.param<std::string>("bus_service_name", bus_service_name, "bus_op");
    service_client = private_nh.serviceClient<arebot_control::BusOp>(bus_service_name);
}

uint16_t AREBot_ROS::Proto::Modbus::modbus_rtu_crc(const uint8_t *buf, uint8_t len) {
    unsigned int temp, flag;
    temp = 0xFFFF;
    for (unsigned char i = 0; i < len; i++) {
        temp = temp ^ buf[i];
        for (unsigned char j = 1; j <= 8; j++) {
            flag = temp & 0x0001;
            temp >>= 1;
            if (flag)
                temp ^= 0xA001;
        }
    }
    temp &= 0xFFFF;
    return temp;
}

bool AREBot_ROS::Proto::Modbus::read_registers(uint8_t addr, uint8_t func_code, uint16_t start_addr, uint16_t len,
                                                std::vector<uint16_t> &ret_buf) {
    arebot_control::BusOp bus_op;
    uint16_t rtu_crc;
    bus_op.request.bus_id = bus_id;
    bus_op.request.data.resize(8);
    bus_op.request.data[0] = addr;
    bus_op.request.data[1] = func_code;
    bus_op.request.data[2] = start_addr >> 8;
    bus_op.request.data[3] = start_addr & 0xff;
    bus_op.request.data[4] = len >> 8;
    bus_op.request.data[5] = len & 0xff;
    rtu_crc = modbus_rtu_crc(bus_op.request.data.data(), 6);
    bus_op.request.data[6] = rtu_crc & 0xff;
    bus_op.request.data[7] = rtu_crc >> 8;
    bus_op.request.recv_len = 5 + len * 2;

    if (!call_service(bus_op))
        return false;

    // check returned crc
    rtu_crc = bus_op.response.data[bus_op.response.data.size() - 1];
    rtu_crc = (rtu_crc << 8) | bus_op.response.data[bus_op.response.data.size() - 2];
    if (rtu_crc != modbus_rtu_crc(bus_op.response.data.data(), bus_op.response.data.size() - 2)) {
        ROS_WARN("CRC mismatch.");
        return false;
    }

    if ((bus_op.response.data[0] != addr) || (bus_op.response.data[1] != func_code) ||
        (bus_op.response.data[2] != len * 2)) {
        ROS_WARN("Mangled data???");
        return false;
    }

    ret_buf.resize(len);

    for (int i = 0; i < len; i++)
        ret_buf[i] = (((uint16_t) bus_op.response.data[3 + i * 2]) << 8) | bus_op.response.data[3 + i * 2 + 1];
    return true;
}

bool AREBot_ROS::Proto::Modbus::preset_register(uint8_t addr, uint16_t reg_addr, uint16_t val) {
    arebot_control::BusOp bus_op;
    uint16_t rtu_crc;
    bus_op.request.bus_id = bus_id;
    bus_op.request.data.resize(8);
    bus_op.request.data[0] = addr;
    bus_op.request.data[1] = 0x06;
    bus_op.request.data[2] = reg_addr >> 8;
    bus_op.request.data[3] = reg_addr & 0xff;
    bus_op.request.data[4] = val >> 8;
    bus_op.request.data[5] = val & 0xff;
    rtu_crc = modbus_rtu_crc(bus_op.request.data.data(), 6);
    bus_op.request.data[6] = rtu_crc & 0xff;
    bus_op.request.data[7] = rtu_crc >> 8;
    if (addr)
        bus_op.request.recv_len = 8;
    else
        bus_op.request.recv_len = 0;
    if (!call_service(bus_op))
        return false;
    if (!addr)
        return true;
    // response should be exactly the same as request
    return bus_op.request.data == bus_op.response.data;
}

bool AREBot_ROS::Proto::Modbus::preset_registers(uint8_t addr, uint16_t reg_addr, const std::vector<uint16_t> &val) {
    arebot_control::BusOp bus_op;
    uint16_t rtu_crc;
    if (9 + val.size() * 2 > 0xff)
        return false;
    bus_op.request.bus_id = bus_id;
    bus_op.request.data.resize(9 + val.size() * 2);
    bus_op.request.data[0] = addr;
    bus_op.request.data[1] = 0x10;
    bus_op.request.data[2] = reg_addr >> 8;
    bus_op.request.data[3] = reg_addr & 0xff;
    bus_op.request.data[4] = val.size() >> 8;
    bus_op.request.data[5] = val.size() & 0xff;
    bus_op.request.data[6] = val.size() * 2;
    size_t buf_ptr = 7;
    for (auto item : val) {
        bus_op.request.data[buf_ptr++] = item >> 8;
        bus_op.request.data[buf_ptr++] = item & 0xff;
    }
    rtu_crc = modbus_rtu_crc(bus_op.request.data.data(), buf_ptr);
    bus_op.request.data[buf_ptr++] = rtu_crc & 0xff;
    bus_op.request.data[buf_ptr++] = rtu_crc >> 8;
    if (addr)
        bus_op.request.recv_len = 8;
    else
        bus_op.request.recv_len = 0;
    if (!call_service(bus_op))
        return false;
    if (!addr)
        return true;
    if(memcmp(bus_op.request.data.data(), bus_op.response.data.data(), 6))
        return false;
    rtu_crc = bus_op.response.data[7];
    rtu_crc = (rtu_crc << 8) | bus_op.response.data[6];

    if (rtu_crc != modbus_rtu_crc(bus_op.response.data.data(), 6)) {
        ROS_WARN("CRC mismatch.");
        return false;
    }
    return true;
}