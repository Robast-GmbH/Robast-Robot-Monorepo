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
      serial_print_error("Failed to mount LittleFS");
    }

    serial_print_green("Successfully mounted LittleFS");
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

    if (_current_file_number >= (max_files - 1))
    {
      // Maximum number of log files reached. Starting from the beginning
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
      serial_print_error("Failed to create new log file");
      return;
    }
    file.close();
  }

  void RotatingFileHandler::write(std::string msg)
  {
    File file = LittleFS.open(_current_file_name, FILE_APPEND);
    if (!file)
    {
      serial_print_error("Failed to open log file!");
      return;
    }

    const uint32_t file_size = file.size();
    file.close();

    if (file_size > _config->get_max_file_size_in_bytes())
    {
      rotate_logs();
    }

    file = LittleFS.open(_current_file_name, FILE_APPEND);

    if (!file.println(msg.c_str()))
    {
      serial_print_error("Failed to write to log file!");
      return;
    }

    serial_print_green("Successfully wrote to log file " + std::string(_current_file_name) +
                       ". Current file size: " + std::to_string(file_size));
  }

  void RotatingFileHandler::print_all_logs()
  {
    serial_print_green("Printing all logs! The content of ...");

    std::vector<String> log_files = get_all_logs();

    uint16_t current_log_file_num = 0;

    for (const auto &log_file : log_files)
    {
      File file = LittleFS.open(log_file.c_str(), "r");
      if (!file)
      {
        serial_print_error("Failed to open log file " + std::string(log_file.c_str()));
        return;
      }

      serial_print_green("... log file " + std::string(log_file.c_str()) + " is:");
      while (file.available())
      {
        Serial.print(ANSI_COLOR_CYAN);
        Serial.printf("%c", file.read());
        Serial.print(ANSI_COLOR_RESET);
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
      LittleFS.remove(file_name);
    }
  }

  void RotatingFileHandler::serial_print_error(const std::string msg)
  {
    Serial.print(ANSI_COLOR_RED);
    Serial.printf("[RotatingFileHandler]: %s\n", msg.c_str());
    Serial.print(ANSI_COLOR_RESET);
  }

  void RotatingFileHandler::serial_print_green(const std::string msg)
  {
    Serial.print(ANSI_COLOR_GREEN);
    Serial.printf("[RotatingFileHandler]: %s\n", msg.c_str());
    Serial.print(ANSI_COLOR_RESET);
  }
}   // namespace logging