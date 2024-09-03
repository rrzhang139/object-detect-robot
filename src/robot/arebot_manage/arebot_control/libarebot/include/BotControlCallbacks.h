/**
 * Copyright 2021 Chuanhong Guo <gch981213@gmail.com>
 */
/*changed for arebot in 2022.9*/
#ifndef LIBAREBOT_BOTCONTROLCALLBACKS_H
#define LIBAREBOT_BOTCONTROLCALLBACKS_H

#include <cstdint>
#include <vector>

namespace AREBot {
    struct IMUData {
        struct AngularVelocity {
            double x, y, z;
        } angular_velocity;
        struct LinearAcceleration {
            double x, y, z;
        } linear_acceleration;
        struct Quaternion {
            double w, x, y, z;
        } orientation;
    };

    class BotControlCallbacks {
    public:
        virtual void imu_data(const IMUData &data) = 0;

        virtual void battery_voltage(float voltage) = 0;

        virtual void bumper_status(uint32_t status) = 0;

        virtual void bus_response(uint8_t bus_id, const std::vector<char> &buf) = 0;

        virtual ~BotControlCallbacks() {};
    };
}


#endif //LIBAREBOT_BOTCONTROLCALLBACKS_H
