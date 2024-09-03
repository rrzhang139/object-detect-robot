/**
 * Copyright 2021 Chuanhong Guo <gch981213@gmail.com>
 */
/*changed for arebot in 2022.9*/

#include "Serial.h"
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <cstring>
#include <cerrno>
#include <stdexcept>
#include <thread>
#include <chrono>

static speed_t get_baud(int baud) {
    switch (baud) {
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
        case 500000:
            return B500000;
        case 576000:
            return B576000;
        case 921600:
            return B921600;
        case 1000000:
            return B1000000;
        case 1152000:
            return B1152000;
        case 1500000:
            return B1500000;
        case 2000000:
            return B2000000;
        case 2500000:
            return B2500000;
        case 3000000:
            return B3000000;
        case 3500000:
            return B3500000;
        case 4000000:
            return B4000000;
        default:
            throw std::invalid_argument("unsupported baud rate.");
    }
}

AREBot::Serial::~Serial() {
    if (serial_fd > 0)
        close(serial_fd);
}

void AREBot::Serial::open(const char *port, int baud, bool flush_rx) {
    int ret;
    struct termios tty;
    // Use O_NDELAY to ignore DCD state
    serial_fd = ::open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd < 0) {
        throw std::runtime_error(strerror(errno));
    }

    // Disallow other processes to open the port.
    if (ioctl(serial_fd, TIOCEXCL) < 0)
        throw std::runtime_error(strerror(errno));

    /* Ensure that we use blocking I/O */
    ret = fcntl(serial_fd, F_GETFL);
    if (ret == -1) {
        throw std::runtime_error(strerror(errno));
    }

    ret = fcntl(serial_fd, F_SETFL, ret & ~O_NONBLOCK);
    if (ret != 0) {
        throw std::runtime_error(strerror(errno));
    }

    if (tcgetattr(serial_fd, &tty) != 0) {
        throw std::runtime_error(strerror(errno));
    }

    speed_t baud_option = get_baud(baud);
    cfsetospeed(&tty, baud_option);
    cfsetispeed(&tty, baud_option);

    tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    tty.c_cflag |= (CS8 | CLOCAL | CREAD);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | IEXTEN);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | IGNCR | INLCR);
    tty.c_oflag &= ~OPOST;

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        throw std::runtime_error(strerror(errno));
    }

    if (flush_rx) {
        std::this_thread::sleep_for(std::chrono::seconds(2));
        ret = tcflush(serial_fd, TCIOFLUSH);
        if (ret != 0) {
            throw std::runtime_error(strerror(errno));
        }
    }
}

int AREBot::Serial::poll_read() {
    int ret;
    pollfd pfd = {serial_fd, POLLIN, 0};
    ret = poll(&pfd, 1, timeout);
    if (ret <= 0)
        return -1;
    return (pfd.revents & POLLIN) ? 0 : -1;
}

/**
 * Read from serial
 *
 * Note: CDC serial on STM32 doesn't always return the full buffer.
 * We need to retry if we didn't get enough data back.
 * @param buf
 * @param len
 */
int AREBot::Serial::serial_read(char *buf, size_t len) {
    while (len) {
        if (poll_read())
            return -1;
        size_t ret = read(serial_fd, buf, len);
        if (!ret)
            return -1;
        buf += ret;
        len -= ret;
    }
    return 0;
}

void AREBot::Serial::serial_write(const char *buf, size_t len) {
    size_t ret = write(serial_fd, buf, len);
    if (ret != len)
        throw std::runtime_error(strerror(errno));
}