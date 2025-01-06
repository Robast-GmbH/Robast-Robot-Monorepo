#ifndef DRAWER_CONTROLLER_DEBUG_HPP
#define DRAWER_CONTROLLER_DEBUG_HPP

#ifndef RUNNING_TESTS
#include <Arduino.h>
#endif

#include "logging/rotating_file_handler.hpp"

// #define DEBUG

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

#ifdef DEBUG

#ifndef debug_print
#define debug_print(x) Serial.print(x)
#endif

#ifndef debug_print_with_base
#define debug_print_with_base(num, base) Serial.print(num, base)
#endif

#ifndef debug_println
#define debug_println(num) Serial.println(num)
#endif

#ifndef debug_println_with_base
#define debug_println_with_base(num, base) Serial.println(num, base)
#endif

#ifndef debug_printf
#define debug_printf(format, ...) Serial.printf(format, ##__VA_ARGS__)
#endif

#ifndef debug_printf_color
#define debug_printf_color(color, format, ...) \
  Serial.print(color);                         \
  Serial.printf(format, ##__VA_ARGS__);        \
  Serial.print(ANSI_COLOR_RESET)
#endif

#ifndef debug_printf_green
#define debug_printf_green(format, ...) \
  Serial.print(ANSI_COLOR_GREEN);       \
  Serial.printf(format, ##__VA_ARGS__); \
  Serial.print(ANSI_COLOR_RESET)
#endif

#ifndef debug_printf_warning
#define debug_printf_warning(format, ...) \
  Serial.print(ANSI_COLOR_YELLOW);        \
  Serial.printf(format, ##__VA_ARGS__);   \
  Serial.print(ANSI_COLOR_RESET)
#endif

#ifndef debug_begin
#define debug_begin(baudrate) Serial.begin(baudrate)
#endif

#ifndef debug_setup
#define debug_setup(baudrate) serial_setup(baudrate)
#endif

#else

#ifndef debug_print
#define debug_print(x)
#endif

#ifndef debug_print_with_base
#define debug_print_with_base(num, base)
#endif

#ifndef debug_println
#define debug_println(num)
#endif

#ifndef debug_println_with_base
#define debug_println_with_base(num, base)
#endif

#ifndef debug_printf
#define debug_printf(format, ...)
#endif

#ifndef debug_printf_color
#define debug_printf_color(color, format, ...)
#endif

#ifndef debug_printf_green
#define debug_printf_green(format, ...)
#endif

#ifndef debug_printf_warning
#define debug_printf_warning(format, ...)
#endif

#ifndef debug_begin
#define debug_begin(baudrate)
#endif

#ifndef debug_setup
#define debug_setup(baudrate)
#endif

#endif

extern std::shared_ptr<logging::RotatingFileHandler> rotating_file_logger;

#ifndef serial_printf_green
#define serial_printf_green(format, ...) \
  Serial.print(ANSI_COLOR_GREEN);        \
  Serial.printf(format, ##__VA_ARGS__);  \
  Serial.print(ANSI_COLOR_RESET)
#endif

#ifndef serial_printf_color
#define serial_printf_color(color, format, ...) \
  Serial.print(color);                          \
  Serial.printf(format, ##__VA_ARGS__);         \
  Serial.print(ANSI_COLOR_RESET)
#endif

#ifndef serial_printf_warning
#define serial_printf_warning(format, ...) \
  Serial.print(ANSI_COLOR_YELLOW);         \
  Serial.printf(format, ##__VA_ARGS__);    \
  Serial.print(ANSI_COLOR_RESET)
#endif

#ifndef serial_println_warning
#define serial_println_warning(num) \
  Serial.print(ANSI_COLOR_YELLOW);  \
  Serial.println(num);              \
  Serial.print(ANSI_COLOR_RESET)
#endif

#ifndef serial_printf_error
#define serial_printf_error(format, ...) \
  Serial.print(ANSI_COLOR_RED);          \
  Serial.printf(format, ##__VA_ARGS__);  \
  Serial.print(ANSI_COLOR_RESET);        \
  rotating_file_logger->write(format_string(format, ##__VA_ARGS__));
#endif

#ifndef serial_println_error
#define serial_println_error(num) \
  Serial.print(ANSI_COLOR_RED);   \
  Serial.println(num);            \
  Serial.print(ANSI_COLOR_RESET); \
  rotating_file_logger->write(std::string(num) + "\n");
#endif

#ifndef RUNNING_TESTS
inline void serial_setup(unsigned long baudrate)
{
  debug_begin(115200);   // Init serial port and set baudrate
  while (!Serial)
  {
  }
}
#endif

inline std::string format_string(const char* format, ...)
{
  va_list args;
  va_start(args, format);

  // Calculate the size of the formatted string
  va_list args_copy;
  va_copy(args_copy, args);
  const int size = std::vsnprintf(nullptr, 0, format, args_copy);
  va_end(args_copy);

  // Create a string with the required size
  std::vector<char> buffer(size + 1);
  std::vsnprintf(buffer.data(), buffer.size(), format, args);

  va_end(args);

  return std::string(buffer.data(), buffer.size() - 1);
}

#endif   // DRAWER_CONTROLLER_DEBUG_HPP
