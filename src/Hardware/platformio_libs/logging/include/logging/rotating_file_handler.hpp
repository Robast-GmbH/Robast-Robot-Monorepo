#ifndef LOGGING_ROTATING_FILE_HANDLER_HPP
#define LOGGING_ROTATING_FILE_HANDLER_HPP

#include <LittleFS.h>

#include <cstdint>
#include <memory>

#include "FS.h"
#include "logging/rotating_file_handler_config.hpp"

#define FORMAT_LITTLEFS_IF_FAILED true
#define FILE_READ                 "r"
#define FILE_WRITE                "w"
#define FILE_APPEND               "a"

namespace logging
{
  class RotatingFileHandler
  {
   public:
    RotatingFileHandler(const std::shared_ptr<RotatingFileHandlerConfig> config);

    void init();

    void write(const char *msg);

    void print_all_logs();

   private:
    const std::shared_ptr<RotatingFileHandlerConfig> _config;

    uint32_t _current_file_size = 0;
    uint32_t _current_file_number = 0;

    char *_current_file_name = "/log_0.txt";

    void rotate_logs();
  };

}   // namespace logging

#endif   // LOGGING_ROTATING_FILE_HANDLER_HPP