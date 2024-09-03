/**
 * Copyright 2021 Chuanhong Guo <gch981213@gmail.com>
 */

/*changed for arebot in 2022.9*/
#include <BotControl.h>
#include <BotRequest.pb.h>
#include <BotResponse.pb.h>
#include <thread>
#include <chrono>
#include <string>
#include <vector>

// max data length responded from board
#define RCV_MAX_LEN 256

AREBot::BotControl::BotControl(BotControlCallbacks &callbacks) : resp_callbacks(callbacks) {
    GOOGLE_PROTOBUF_VERIFY_VERSION;
}

// The Protocol Buffer wire format is not self-delimiting, and we need to split packets from stream ourselves.
// Prefix each packet with a VarInt encoded packet length using CodedInputStream/CodedOutputStream.

static int base128_varint_recv_decode(AREBot::Serial &s, size_t *val) {
    int shift = 0;
    int ret;
    unsigned char tmp = 0;

    *val = 0;
    ret = s.serial_read((char *) &tmp, 1);
    while ((!ret) && (tmp & 0x80)) {
        *val |= (((size_t) tmp & 0x7f) << shift);
        ret = s.serial_read((char *) &tmp, 1);
        shift += 7;
    }

    *val |= (((size_t) tmp & 0x7f) << shift);
    return ret;
}

static int base128_varint_encode_send(AREBot::Serial &s, size_t num) {
    unsigned char tmp;
    do {
        tmp = num & 0x7f;
        num >>= 7;
        if (num)
            tmp |= 0x80;
        s.serial_write((char *) &tmp, 1);
    } while (num);

    return 0;
}

// simple 16-bit checksum
static uint16_t proto_checksum(const char *buf, size_t len) {
    uint16_t ret = 0;
    for (size_t i = 0; i < len; i++)
        ret += (unsigned char) buf[i];
    return ret;
}

int AREBot::BotControl::send_request_unlocked(BotRequest &req) {
    int ret;
    if (!req.SerializeToString(&pb_buf))
        return -1;
    uint16_t csum = proto_checksum(pb_buf.c_str(), pb_buf.size());
    ret = base128_varint_encode_send(serial, pb_buf.size());
    if (ret)
        return ret;
    pb_buf.append(1, (char) (csum >> 8));
    pb_buf.append(1, (char) (csum & 0xff));
    serial.serial_write(pb_buf.c_str(), pb_buf.size());
    return 0;
}

int AREBot::BotControl::recv_response(BotResponse &resp) {
    size_t len;
    if (base128_varint_recv_decode(serial, &len))
        return -1;
    if (len > RCV_MAX_LEN)
        return -1;
    rcv_buf.resize(len + 2);
    if (serial.serial_read(rcv_buf.data(), len + 2))
        return -1;
    uint16_t csum = (rcv_buf[len] << 8) | (uint8_t) rcv_buf[len + 1];
    uint16_t actual_csum = proto_checksum(rcv_buf.data(), len);
    if (csum != actual_csum)
        return -2;
    bool result = resp.ParseFromArray(rcv_buf.data(), len);
    return result ? 0 : -1;

}

int AREBot::BotControl::process_response() {
    BotResponse response;
    int ret = recv_response(response);
    if (ret)
        return ret;

    switch (response.body_case()) {
        case BotResponse::BodyCase::kBatteryVoltage:
            resp_callbacks.battery_voltage(response.battery_voltage().voltage());
            break;
        case BotResponse::BodyCase::kBumperStatus:
            resp_callbacks.bumper_status(response.bumper_status().status());
            break;
        case BotResponse::BodyCase::kImuData:
            if (imu_ready) {
                const struct AREBot::IMUData data = {
                        .angular_velocity = {
                                .x = response.imu_data().angular_velocity().x() / gyroscope_sensitivity,
                                .y = response.imu_data().angular_velocity().y() / gyroscope_sensitivity,
                                .z = response.imu_data().angular_velocity().z() / gyroscope_sensitivity,
                        },
                        .linear_acceleration = {
                                .x = response.imu_data().linear_acceleration().x() / accelerometer_sensitivity,
                                .y = response.imu_data().linear_acceleration().y() / accelerometer_sensitivity,
                                .z = response.imu_data().linear_acceleration().z() / accelerometer_sensitivity,
                        },
                        .orientation = {
                                .w = response.imu_data().orientation().w() / quaternion_sensitivity,
                                .x = response.imu_data().orientation().x() / quaternion_sensitivity,
                                .y = response.imu_data().orientation().y() / quaternion_sensitivity,
                                .z = response.imu_data().orientation().z() / quaternion_sensitivity,
                        },
                };
                resp_callbacks.imu_data(data);
            }
            break;
        case BotResponse::BodyCase::kBusResponse:
            if (response.bus_response().data_len() > RCV_MAX_LEN)
                return -1;
            rcv_buf.resize(response.bus_response().data_len());
            serial.serial_read(rcv_buf.data(), response.bus_response().data_len());
            resp_callbacks.bus_response(response.bus_response().bus_id(), rcv_buf);
            break;
        case BotResponse::BodyCase::kImuInitResult:
            if (response.imu_init_result().init_result() == 0) {
                gyroscope_sensitivity = response.imu_init_result().gyroscope_sensitivity();
                accelerometer_sensitivity = response.imu_init_result().accelerometer_sensitivity();
                quaternion_sensitivity = response.imu_init_result().quaternion_sensitivity();
                imu_ready = true;
            } else {
                throw std::runtime_error("IMU init failed.");
            }
            break;
        default:
            break;
    }
    return 0;
}

int AREBot::BotControl::imu_init(int data_rate, bool self_test) {
    BotRequest request;
    IMUInit *imu_init;
    imu_init = request.mutable_imu_init();
    imu_init->set_self_test(self_test);
    imu_init->set_report_frequency(data_rate);
    return send_request(request);
}

int AREBot::BotControl::bus_baudrate(uint32_t bus_id, uint32_t baudrate, char paritybit, uint8_t stopbit) {
    BotRequest request;
    BusBaudrate *rate;
    rate = request.mutable_bus_baudrate();
    rate->set_bus_id(bus_id);
    rate->set_baudrate(baudrate);
    if (paritybit == 'E' && stopbit == 1)
        rate->set_parity(BusBaudrate_Parity_PAR_8E1);
    else if (paritybit == 'N' && stopbit == 2)
        rate->set_parity(BusBaudrate_Parity_PAR_8N2);
    else if (paritybit == 'N' && stopbit == 1)
        rate->set_parity(BusBaudrate_Parity_PAR_8N1);
    return send_request(request);
}

int AREBot::BotControl::bus_request(uint8_t bus_id, uint8_t txlen, uint8_t rxlen, const uint8_t *txbuf) {
    BotRequest request;
    BusRequest *bus_request;
    // wait IMU init for up to 5 secs. IMU init routine in control board blocks all other requests.
    for (int i = 0; i < 5 && !imu_ready; i++)
        std::this_thread::sleep_for(std::chrono::seconds(2));
    bus_request = request.mutable_bus_request();
    bus_request->set_bus_id(bus_id);
    bus_request->set_tx_len(txlen);
    bus_request->set_rx_len(rxlen);
    request_mutex.lock();
    int ret = send_request_unlocked(request);
    if (!ret) {
        serial.serial_write((const char *) txbuf, txlen);
    }
    request_mutex.unlock();
    return ret;
}

int AREBot::BotControl::oled_clear() {
    BotRequest request;
    OLEDWrite *oled;
    oled = request.mutable_oled_write();
    oled->set_data_len(0);
    return send_request(request);
}

int AREBot::BotControl::oled_write_string(uint8_t page, uint8_t col, const char *str) {
    size_t len = strlen(str);
    BotRequest request;
    OLEDWrite *oled;
    oled = request.mutable_oled_write();
    oled->set_data_len(len);
    oled->set_page_start(page);
    oled->set_col_start(col);
    oled->set_is_string(true);
    int ret = send_request(request);
    if (ret)
        return ret;
    serial.serial_write(str, len);
    return 0;
}

int AREBot::BotControl::oled_write_raw(uint8_t page_s, uint8_t page_e, uint8_t col_s, uint8_t col_e, uint8_t *buf,
                                        size_t len) {

    BotRequest request;
    OLEDWrite *oled;
    oled = request.mutable_oled_write();
    oled->set_data_len(len);
    oled->set_page_start(page_s);
    oled->set_page_end(page_e);
    oled->set_col_start(col_s);
    oled->set_col_end(col_e);
    oled->set_is_string(false);
    int ret = send_request(request);
    if (ret)
        return ret;
    serial.serial_write(reinterpret_cast<const char *>(buf), len);
    return 0;
}