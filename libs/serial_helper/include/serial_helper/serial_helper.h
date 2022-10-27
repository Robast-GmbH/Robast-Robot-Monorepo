#ifndef SERIAL_HELPER_HPP_
#define SERIAL_HELPER_HPP_

#include <cstring>
#include <string>
// Linux headers for serial communication
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include "i_serial_helper.h"

using namespace std;

namespace serial_helper
{
    class SerialHelper:public ISerialHelper
    {
        private:
            /* data */
            string serial_path_;
            int serial_port_;

        public:
            SerialHelper(string serial_path);
            ~SerialHelper();

            string open_serial();
            void close_serial();

            uint16_t read_serial(string* result, uint16_t max_num_bytes);
            string write_serial(string msg);
            string send_ascii_cmd(string cmd);            
            string ascii_interaction(string cmd, string* responce, uint16_t responce_size );
    };
}

#endif /* SERIAL_HELPER_HPP_ */