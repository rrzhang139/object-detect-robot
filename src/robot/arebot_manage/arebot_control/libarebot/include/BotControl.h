/**
 * Copyright 2021 Chuanhong Guo <gch981213@gmail.com>
 */

/*changed for arebot in 2022.9*/

#ifndef LIBAREBOT_BOTCONTROL_H
#define LIBAREBOT_BOTCONTROL_H

#include <Serial.h>
#include <BotControlCallbacks.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <BotRequest.pb.h>
#include <BotResponse.pb.h>
#include <memory>
#include <string>
#include <mutex>
#include <fstream>

namespace AREBot {
    class BotControl {
    private:
        Serial serial;
        BotControlCallbacks &resp_callbacks;
        bool imu_ready = false;
        float gyroscope_sensitivity = 0;
        float accelerometer_sensitivity = 0;
        float quaternion_sensitivity = 0;
        std::mutex request_mutex;
        std::string pb_buf;
        std::vector<char> rcv_buf;

        int send_request_unlocked(BotRequest &req);

        int send_request(BotRequest &req) {
            int ret;
            request_mutex.lock();
            ret = send_request_unlocked(req);
            request_mutex.unlock();
            return ret;
        }

        int recv_response(BotResponse &resp);

    public:
        BotControl(BotControlCallbacks &callbacks);

        void open(const char *port, int baud, bool flush_rx = true) {
            serial.open(port, baud, flush_rx);
        }

        int process_response();

        int imu_init(int data_rate, bool self_test = false);

        int bus_baudrate(uint32_t bus_id, uint32_t baudrate, char paritybit, uint8_t stopbit);

        int bus_request(uint8_t bus_id, uint8_t txlen, uint8_t rxlen, const uint8_t *txbuf);

        int oled_clear();

        int oled_write_string(uint8_t page, uint8_t col, const char *str);

        int oled_write_raw(uint8_t page_s, uint8_t page_e, uint8_t col_s, uint8_t col_e, uint8_t *buf, size_t len);
    };
}

#endif //LIBARBOT_BOTCONTROL_H
