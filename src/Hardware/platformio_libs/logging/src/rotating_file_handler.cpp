#include "logging/rotating_file_handler.hpp"

namespace logging
{
  RotatingFileHandler::RotatingFileHandler(const std::shared_ptr<RotatingFileHandlerConfig> config) : _config(config)
  {
    init();
  }

  void RotatingFileHandler::init()
  {
    if (!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED))
    {
      serial_printf_error("[RotatingFileHandler]: Failed to mount LittleFS\n");
    }
    serial_printf_green("[RotatingFileHandler]: Successfully mounted LittleFS\n");
  }

  void RotatingFileHandler::rotate_logs()
  {
    debug_printf_green("[RotatingFileHandler]: Rotating log files\n");

    const uint8_t max_files = _config->get_max_files();

    // Close the current log file if it is open
    File file = LittleFS.open(_current_file_name, FILE_READ);
    if (file)
    {
      file.close();
    }

    if (_current_file_number >= (max_files - 1))
    {
      debug_printf_green("[RotatingFileHandler]: Maximum number of log files reached. Starting from the beginning\n");
      _current_file_number = 0;
    }
    else
    {
      ++_current_file_number;
    }

    snprintf(_current_file_name, sizeof(_current_file_name), "/log_%d.txt", _current_file_number);

    delete_log_file_if_exists(_current_file_name);

    // Create a new log file
    file = LittleFS.open(_current_file_name, FILE_WRITE);
    if (!file)
    {
      serial_printf_error("[RotatingFileHandler]: Failed to create new log file\n");
      return;
    }
    file.close();

    debug_printf_green("[RotatingFileHandler]: Log file rotated successfully\n");
  }

  void RotatingFileHandler::write(std::string msg)
  {
    File file = LittleFS.open(_current_file_name, FILE_APPEND);
    if (!file)
    {
      serial_printf_error("[RotatingFileHandler]: Failed to open log file\n");
      return;
    }

    file.println(msg.c_str());
    const uint32_t file_size = file.size();
    file.close();

    if (file_size > _config->get_max_file_size_in_bytes())
    {
      serial_printf_color(
        ANSI_COLOR_YELLOW,
        "[RotatingFileHandler]: Current Log file size %d exceeds maximum file size %d. Rotating log files\n",
        file_size,
        _config->get_max_file_size_in_bytes());
      rotate_logs();
    }

    debug_printf_green("[RotatingFileHandler]: Successfully wrote to log file %s. Current file size: %d\n",
                       _current_file_name,
                       file_size);
  }

  void RotatingFileHandler::print_all_logs()
  {
    std::vector<String> log_files = get_all_logs();

    uint16_t current_log_file_num = 0;

    // Print all log files
    for (const auto &log_file : log_files)
    {
      // print content of all log files
      File file = LittleFS.open(log_file.c_str(), "r");
      if (!file)
      {
        serial_printf_error("[RotatingFileHandler]: Failed to open log file %s\n", log_file.c_str());
        return;
      }

      serial_printf_green("[RotatingFileHandler]: Printing content of log file %s:\n", log_file.c_str());
      // print content of file
      while (file.available())
      {
        serial_printf_color(ANSI_COLOR_CYAN, "%c", file.read());
      }
      file.close();

      if (current_log_file_num >= _config->get_max_files())
      {
        delete_log_file_if_exists(log_file.c_str());
      }

      ++current_log_file_num;
    }
  }

  std::vector<String> RotatingFileHandler::get_all_logs() const
  {
    std::vector<String> log_files;
    File root = LittleFS.open("/");
    File f = root.openNextFile();
    while (f)
    {
      log_files.push_back(String("/") + f.name());
      f = root.openNextFile();
    }

    return log_files;
  }

  void RotatingFileHandler::delete_log_file_if_exists(const char *file_name)
  {
    if (LittleFS.exists(file_name))
    {
      debug_printf_green("[RotatingFileHandler]: Deleting existing log file %s\n", file_name);
      LittleFS.remove(file_name);
    }
  }

}   // namespace logging