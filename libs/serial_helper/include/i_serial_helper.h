#ifndef I_SERIAL_HELPER_HPP_
#define I_SERIAL_HELPER_HPP_

#include <cstring>
#include <string>

using namespace std;

namespace serial_helper
{
    class ISerialHelper
    {
        

        public:
            
            virtual string open_serial()= 0;
            virtual void close_serial() = 0;

            virtual uint16_t read_serial(string* result, uint16_t max_num_bytes)=0;

            virtual string write_serial(string msg) = 0;

            virtual string send_ascii_cmd(string cmd) = 0;

            virtual string ascii_interaction(string cmd, string* responce, uint16_t responce_size )=0;
    };
}

#endif /* I_SERIAL_HELPER_HPP_ */