#ifndef HARDWARE_NODES__I_SERIAL_HELPER_HPP_
#define HARDWARE_NODES__I_SERIAL_HELPER_HPP_

#include <cstring>
#include <string>

namespace serial_helper
{
  class ISerialHelper
  {
   public:
    virtual std::string open_serial() = 0;
    virtual void close_serial() = 0;

    virtual uint16_t read_serial(std::string& result, uint16_t max_num_bytes) = 0;

    virtual std::string write_serial(std::string msg) = 0;

    virtual std::string send_ascii_cmd(std::string cmd) = 0;

    virtual uint16_t ascii_interaction(const std::string cmd,
                                       std::string& response,
                                       uint16_t response_max_size = 100) = 0;
  };
}   // namespace serial_helper

#endif /* HARDWARE_NODES__I_SERIAL_HELPER_HPP_ */