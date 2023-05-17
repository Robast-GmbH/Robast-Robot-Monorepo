#include "test/mock_serial_helper.hpp"

namespace serial_helper
{

    MockSerialHelper::MockSerialHelper(std::string key)
    {
        key_code_ = key;
    }

    MockSerialHelper::~MockSerialHelper()
    {

    }

    std::string MockSerialHelper::open_serial()
    {
        return "0001";
    }

    void MockSerialHelper::close_serial()
    {

    }

    uint16_t MockSerialHelper::read_serial(std::string* result, uint16_t max_num_bytes)
    {
        *result = "0001";
        return 4;
    }

    std::string MockSerialHelper::write_serial(std::string msg)
    {
        if (msg == NFC_READ_MC("02"))
        {

            return key_code_;
        }
        return "0001";
    }

    std::string MockSerialHelper::send_ascii_cmd(std::string cmd)
    {
        if (cmd == NFC_READ_MC("02"))
        {

            return key_code_;
        }
        return "0001";
    }

    std::string MockSerialHelper::ascii_interaction(std::string cmd, std::string* response, uint16_t response_size)
    {
        
        (void)response_size;
         if (cmd == NFC_READ_MC("02"))
        {
            *response = key_code_;
        }
        else if (cmd == DEVICE_STATE)
        {
            *response = RESPONCE_DEVICE_STATE_CONFIGURED;
        }
        else
        {
            *response = "0001";
            
        }
        return *response ;
    }

}