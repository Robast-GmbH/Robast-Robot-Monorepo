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

  uint16_t MockSerialHelper::read_serial(std::string* result, uint16_t)
  {
    *result = key_code_;
    return key_code_.length();
  }

  std::string MockSerialHelper::write_serial(std::string)
  {
    return key_code_;
  }

  std::string MockSerialHelper::send_ascii_cmd(std::string)
  {
    return "";
  }

  std::string MockSerialHelper::ascii_interaction(std::string, std::string* response, uint16_t response_size)
  {
    if (response_size > key_code_.size())
    {
      *response = key_code_;
    }
    return *response;
  }

}   // namespace serial_helper