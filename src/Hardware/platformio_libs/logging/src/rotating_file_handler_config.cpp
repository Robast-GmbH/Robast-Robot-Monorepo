#include "logging/rotating_file_handler_config.hpp"

namespace logging
{
  void RotatingFileHandlerConfig::print_all_configs() const
  {
    Serial.print(ANSI_COLOR_BLUE);
    Serial.printf("[RotatingFileHandlerConfig]: Max file size: %u\n", _max_file_size);
    Serial.printf("[RotatingFileHandlerConfig]: Max files: %u\n", _max_files);
    Serial.print(ANSI_COLOR_RESET);
  }

  void RotatingFileHandlerConfig::set_max_file_size_in_bytes(const uint16_t max_file_size)
  {
    _max_file_size = max_file_size;
  }

  void RotatingFileHandlerConfig::set_max_files(const uint8_t max_files)
  {
    _max_files = max_files;
  }

  uint16_t RotatingFileHandlerConfig::get_max_file_size_in_bytes() const
  {
    return _max_file_size;
  }

  uint8_t RotatingFileHandlerConfig::get_max_files() const
  {
    return _max_files;
  }

}   // namespace logging