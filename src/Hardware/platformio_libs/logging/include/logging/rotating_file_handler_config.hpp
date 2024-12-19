#ifndef LOGGING_ROTATING_FILE_HANDLER_CONFIG_HPP
#define LOGGING_ROTATING_FILE_HANDLER_CONFIG_HPP

#include <cstdint>

#include "debug/debug.hpp"

namespace logging
{
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