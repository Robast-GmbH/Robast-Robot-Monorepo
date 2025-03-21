#ifndef LOGGING_ROTATING_FILE_HANDLER_HPP
#define LOGGING_ROTATING_FILE_HANDLER_HPP

#include <LittleFS.h>

#include <cstdint>
#include <memory>

#include "FS.h"
#include "logging/rotating_file_handler_config.hpp"

namespace logging
{
  constexpr bool FORMAT_LITTLEFS_IF_FAILED = true;

  class RotatingFileHandler
  {
   public:
    explicit RotatingFileHandler(const std::shared_ptr<RotatingFileHandlerConfig> config);

    void init();

    void write(std::string msg);

    void print_all_logs();

   private:
    const std::shared_ptr<RotatingFileHandlerConfig> _config;

    uint32_t _current_file_size = 0;
    uint32_t _current_file_number = 0;

    char _current_file_name[20] = "/log_0.txt";

    void rotate_logs();

    void delete_log_file_if_exists(const char* file_name);

    std::vector<String> get_all_logs() const;

    void serial_print_error(const std::string msg);

    void serial_print_green(const std::string msg);
  };

}   // namespace logging

#endif   // LOGGING_ROTATING_FILE_HANDLER_HPP