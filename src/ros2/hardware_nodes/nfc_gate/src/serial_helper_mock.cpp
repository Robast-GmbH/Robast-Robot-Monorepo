#include "nfc_gate/serial_helper_mock.hpp"

namespace robast
{
            
    string MockSerialHelper::open_serial()
    {
        return RESPONCE_SURESPONCE_SUCCESS;
    }

    void MockSerialHelper::close_serial()
    {

    }

    uint16_t MockSerialHelper::read_serial(string* result, uint16_t max_num_bytes)
    {

    }

    string MockSerialHelper::write_serial(string msg) 
    {

    }

    string MockSerialHelper::send_ascii_cmd(string cmd)
    {

    }

    string MockSerialHelper::ascii_interaction(string cmd, string* responce, uint16_t responce_size )
    {

    }
}