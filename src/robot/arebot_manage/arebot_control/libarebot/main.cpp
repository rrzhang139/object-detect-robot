#include <iostream>
#include <memory>
#include <cmath>
#include <getopt.h>
#include <BotControl.h>
#include <stdexcept>

using std::cout;
using std::endl;

const char *serial_port = "/dev/smart_car";
const char *oled_str;
bool flush_rx = true;

static struct option long_options[] = {
        {"port", required_argument, 0, 'p'},
        {"oled", required_argument, 0, 'd'},
        {0, 0,                      0, 0}
};

static enum class op_mode_t {
    OLED,
    TEST
} op_mode = op_mode_t::TEST;

class BotCB : public AREBot::BotControlCallbacks {
public:
    virtual void imu_data(const AREBot::IMUData &data) {
        cout << "Acceleration (x,y,z): " << data.linear_acceleration.x << " " << data.linear_acceleration.y << " "
             << data.linear_acceleration.z << endl;
        cout << "Angular Velocity(x,y,z): " << data.angular_velocity.x << " " << data.angular_velocity.y << " "
             << data.angular_velocity.z << endl;
        cout << "Orientation (w,x,y,z): " << data.orientation.w << " " << data.orientation.x << " "
             << data.orientation.y << " " << data.orientation.z << endl;
        const AREBot::IMUData::Quaternion &q = data.orientation;

        // roll (x-axis rotation)
        double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
        double roll = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2 * (q.w * q.y - q.z * q.x);
        double pitch;
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            pitch = std::asin(sinp);

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        cout << "Orientation (YPR): " << yaw << " " << pitch << " " << roll << endl;
    }

    virtual void battery_voltage(float voltage) {
        cout << "Battery Voltage: " << voltage << endl;
    }

    virtual void bumper_status(uint32_t status) {
    }

    virtual void bus_response(uint8_t bus_id, const std::vector<char> &buf) {
        cout << "We got something from bus " << bus_id << endl;
    }

    virtual ~BotCB() {}
};

static int oled_write(AREBot::BotControl &ctl, const char *str) {
    ctl.oled_clear();
    ctl.oled_write_string(0, 0, str);
    return 0;
}

static int dev_test(AREBot::BotControl &ctl) {
    int tocnt = 0;
    const uint8_t reqbuf[] = {
            0x01, 0x06, 0, 1, 0, 0x32, 0x59, 0xdf
    };

    ctl.imu_init(5, true);
    ctl.bus_request(0, sizeof(reqbuf), sizeof(reqbuf), reqbuf);
    while (1) {
        if (ctl.process_response()) {
            cout << "timed out?" << endl;
            tocnt++;
        } else {
            tocnt = 0;
        }
        if (tocnt > 10)
            break;
    }
    return 0;
}

int main(int argc, char **argv) {
    int c;
    BotCB cb;
    AREBot::BotControl ctl(cb);
    while (((c = getopt_long(argc, argv, "p:d:", long_options, nullptr))) != -1) {
        switch (c) {
            case 'p':
                serial_port = optarg;
                break;
            case 'd':
                op_mode = op_mode_t::OLED;
                oled_str = optarg;
                flush_rx = false;
                break;
            default:
                fputs("unknown option.", stderr);
                return -1;
        }
    }

    try {
        ctl.open(serial_port, 115200, flush_rx);
    } catch (std::runtime_error &err) {
        fprintf(stderr, "failed to open %s: %s\n", serial_port, err.what());
        return -1;
    }
    switch (op_mode) {
        case op_mode_t::OLED:
            return oled_write(ctl, oled_str);
        case op_mode_t::TEST:
            return dev_test(ctl);
    }
    return 0;
}
