#ifndef HARDWARE_NODES__MOCK_SERIAL_HELPER_HPP_
#define HARDWARE_NODES__MOCK_SERIAL_HELPER_HPP_

#include "serial_helper/i_serial_helper.h"

namespace serial_helper
{

  class MockSerialHelper : public ISerialHelper
  {
   public:
    MockSerialHelper();   // added
    MockSerialHelper(std::string key);
    ~MockSerialHelper();

    std::string open_serial();
    void close_serial();
    uint16_t read_serial(std::string* result, uint16_t max_num_bytes);
    std::string write_serial(std::string msg);
    std::string send_ascii_cmd(std::string cmd);
    std::string ascii_interaction(std::string cmd, std::string* response, uint16_t response_size);

   private:
    std::string key_code_;
  };
}   // namespace serial_helper
#endif   // HARDWARE_NODES__MOCK_SERIAL_HELPER_HPP_