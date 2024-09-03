/**
 * Copyright 2021 Chuanhong Guo <gch981213@gmail.com>
 */

/*changed for arebot in 2022.9*/

#ifndef LIBAREBOT_SERIAL_H
#define LIBAREBOT_SERIAL_H
#include <termios.h>
#include <cstdlib>

namespace AREBot {
    class Serial {
    private:
        int serial_fd = -1;
        int timeout;
        int poll_read();
    public:
        explicit Serial(int _timeout = 1000) : timeout(_timeout) {}
        void open(const char *port, int baud, bool flush_rx = true);
        virtual ~Serial();
        int serial_read(char *buf, size_t len);
        void serial_write(const char *buf, size_t len);
    };
}


#endif //LIBAREBOT_SERIAL_H
