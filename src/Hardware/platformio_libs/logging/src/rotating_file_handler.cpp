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
    const uint8_t max_files = _config->get_max_files();

    // Close the current log file if it is open
    File file = LittleFS.open(_current_file_name, FILE_READ);
    if (file)
    {
      file.close();
    }

    if (_current_file_number >= max_files)
    {
      serial_printf_green("[RotatingFileHandler]: Maximum number of log files reached. Deleting oldest log file\n");
      LittleFS.remove("/log_0.txt");
      _current_file_number = 0;
    }
    else
    {
      ++_current_file_number;
    }

    snprintf(_current_file_name, sizeof(_current_file_name), "/log_%d.txt", _current_file_number);

    // Check if the new log file already exists and delete it
    if (LittleFS.exists(_current_file_name))
    {
      LittleFS.remove(_current_file_name);
    }

    // Create a new log file
    file = LittleFS.open(_current_file_name, FILE_WRITE);
    if (!file)
    {
      serial_printf_error("[RotatingFileHandler]: Failed to create new log file\n");
      return;
    }
    file.close();

    serial_printf_green("[RotatingFileHandler]: Log file rotated successfully\n");
  }

  void RotatingFileHandler::write(const char *msg)
  {
    serial_printf_green("[RotatingFileHandler]: Writing to log file %s\n", _current_file_name);

    // write to file here and rotate if necessary
    File file = LittleFS.open(_current_file_name, FILE_APPEND);
    if (!file)
    {
      serial_printf_error("[RotatingFileHandler]: Failed to open log file\n");
      return;
    }

    file.println(msg);
    file.close();

    if (file.size() > _config->get_max_file_size_in_bytes())
    {
      rotate_logs();
    }

    serial_printf_green("[RotatingFileHandler]: Successfully wrote to log file\n");
  }

  void RotatingFileHandler::print_all_logs()
  {
    // Check how many log files exist
    std::vector<String> log_files;
    File root = LittleFS.open("/");
    File f = root.openNextFile();
    while (f)
    {
      log_files.push_back(f.name());
      f = root.openNextFile();
    }

    // Print all log files
    for (const auto &log_file : log_files)
    {
      // print content of all log files
      File file = LittleFS.open("/" + log_file, "r");
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
    }
  }

}   // namespace logging