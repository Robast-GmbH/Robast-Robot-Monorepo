#include "test/mock_serial_helper.hpp"

namespace serial_helper
{

    MockSerialHelper::MockSerialHelper()
    {}

    MockSerialHelper::~MockSerialHelper()
    {}

    string MockSerialHelper::open_serial()
    {}

    void MockSerialHelper::close_serial()
    {}

    uint16_t MockSerialHelper::read_serial(string* result, uint16_t max_num_bytes)
    {}

    string MockSerialHelper::write_serial(string msg)
    {}

    string MockSerialHelper::send_ascii_cmd(string cmd)
    {}

    string MockSerialHelper::ascii_interaction(string cmd, string* response, uint16_t response_size)
    {}

}