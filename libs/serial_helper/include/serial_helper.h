#ifndef SERIAL_HELPER_HPP_
#define SERIAL_HELPER_HPP_

#include <cstring>
#include <string>
// Linux headers for serial communication
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

using namespace std;

namespace serial_helper
{
    class SerialHelper
    {
        private:
            /* data */
            string serial_path;
            int serial_port;

        public:
            SerialHelper(string serial_path);
            ~SerialHelper();

            string open_serial();
            void close_serial();

            uint16_t read_serial(string* result, uint16_t max_num_bytes);

            string write_serial(string msg);

            string send_ascii_cmd(string cmd);
    };
}

#endif /* SERIAL_HELPER_HPP_ */