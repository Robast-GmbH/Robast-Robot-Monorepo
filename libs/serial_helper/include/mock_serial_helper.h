#ifndef MOCK_SERIAL_HELPER_HPP_
#define MOCK_SERIAL_HELPER_HPP_

#include <cstring>
#include <string>
// Linux headers for serial communication
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include "i_serial_helper.h"
#include <map>
#include <queue>

using namespace std;

namespace serial_helper
{
    class MockSerialHelper:public ISerialHelper
    {
        private:
            // <messgaes, Responces>  
            map<string, string > responces;  
            queue<string> responceBuffer; 

        public:
            MockSerialHelper(map<string,string > responces );

            string open_serial();
            void close_serial();

            uint16_t read_serial(string* result, uint16_t max_num_bytes);

            string write_serial(string msg);

            string send_ascii_cmd(string cmd);
    };
}

#endif /* MOCK_SERIAL_HELPER_HPP_ */