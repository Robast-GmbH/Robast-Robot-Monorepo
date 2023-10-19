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
        return "";
    }

    void MockSerialHelper::close_serial()
    {

    }

    uint16_t MockSerialHelper::read_serial(std::string* result, uint16_t max_num_bytes)
    {
        *result = key_code_;
        return key_code_.length();
    }

    std::string MockSerialHelper::write_serial(std::string msg)
    {
            return key_code_;      
    }

    std::string MockSerialHelper::send_ascii_cmd(std::string cmd)
    {
      return "";
    }

    std::string MockSerialHelper::ascii_interaction(std::string cmd, std::string* response, uint16_t response_size)
    {
        
        *response = key_code_;
        return *response ;
    }

}