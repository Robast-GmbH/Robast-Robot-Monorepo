#ifndef DRAWER_CONTROLLER_DEBUG_HPP
#define DRAWER_CONTROLLER_DEBUG_HPP

#include <Arduino.h>

#define DEBUG

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

#ifndef serial_printf_green
#define serial_printf_green(format, ...) \
  Serial.print(ANSI_COLOR_GREEN);        \
  Serial.printf(format, ##__VA_ARGS__);  \
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
  Serial.print(ANSI_COLOR_RESET)
#endif

#ifndef serial_println_error
#define serial_println_error(num) \
  Serial.print(ANSI_COLOR_RED);   \
  Serial.println(num);            \
  Serial.print(ANSI_COLOR_RESET)
#endif

inline void serial_setup(unsigned long baudrate)
{
  debug_begin(115200);   // Init serial port and set baudrate
  while (!Serial)
  {
  }
}

#endif   // DRAWER_CONTROLLER_DEBUG_HPP
