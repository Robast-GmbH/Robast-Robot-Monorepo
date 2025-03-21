#ifndef LOGGING_ROTATING_FILE_HANDLER_CONFIG_HPP
#define LOGGING_ROTATING_FILE_HANDLER_CONFIG_HPP

#ifndef RUNNING_TESTS
#include <Arduino.h>
#endif

#include <cstdint>

namespace logging
{
  constexpr const char* ANSI_COLOR_BLUE = "\x1b[34m";
  constexpr const char* ANSI_COLOR_RED = "\x1b[31m";
  constexpr const char* ANSI_COLOR_GREEN = "\x1b[32m";
  constexpr const char* ANSI_COLOR_CYAN = "\x1b[36m";
  constexpr const char* ANSI_COLOR_RESET = "\x1b[0m";

  class RotatingFileHandlerConfig
  {
   public:
    RotatingFileHandlerConfig() = default;

    void print_all_configs() const;

    void set_max_file_size_in_bytes(const uint16_t max_file_size);
    void set_max_files(const uint8_t max_files);

    uint16_t get_max_file_size_in_bytes() const;
    uint8_t get_max_files() const;

   private:
    uint16_t _max_file_size;
    uint8_t _max_files;
  };

}   // namespace logging

#endif   // LOGGING_ROTATING_FILE_HANDLER_CONFIG_HPP