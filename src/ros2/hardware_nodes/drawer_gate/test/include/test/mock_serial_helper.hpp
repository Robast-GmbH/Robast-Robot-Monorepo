#include "serial_helper/i_serial_helper.h"
#include "can/can_db_defines.h"
#include "can/can_message.h"
#include "can/can_db.hpp"
#include "can/can_helper.h"

namespace serial_helper
{

    class MockSerialHelper : public ISerialHelper
    {
    public:
        MockSerialHelper();
        ~MockSerialHelper();

        string open_serial();
        void close_serial();
        uint16_t read_serial(string* result, uint16_t max_num_bytes);
        string write_serial(string msg);
        string send_ascii_cmd(string cmd);
        string ascii_interaction(string cmd, string* response, uint16_t response_size);

    private:
        string key_code;
    };
}