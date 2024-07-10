#ifndef HARDWARE_NODES__SERIAL_HELPER_H_
#define HARDWARE_NODES__SERIAL_HELPER_H_

#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <regex>
#include <string>
#include <thread>
// Linux headers for serial communication
#include <errno.h>     // Error integer and strerror() function
#include <fcntl.h>     // Contains file controls like O_RDWR
#include <termios.h>   // Contains POSIX terminal control definitions
#include <unistd.h>    // write(), read(), close()

#include "i_serial_helper.hpp"

namespace serial_helper
{
  class SerialHelper : public ISerialHelper
  {
   private:
    /* data */
    std::string serial_path_;
    int serial_port_;

   public:
    explicit SerialHelper(std::string serial_path);
    ~SerialHelper();

    std::string open_serial() override;
    void close_serial() override;

    uint16_t read_serial(std::string& result, uint16_t max_num_bytes) override;
    std::string write_serial(std::string msg) override;
    std::string send_ascii_cmd(std::string cmd) override;
    uint16_t ascii_interaction(const std::string cmd, std::string& response, uint16_t response_max_size = 100) override;
  };
}   // namespace serial_helper

#endif /* HARDWARE_NODES__SERIAL_HELPER_H_ */