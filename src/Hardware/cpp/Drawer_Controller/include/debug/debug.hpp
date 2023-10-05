#ifndef DRAWER_CONTROLLER_DEBUG_HPP
#define DRAWER_CONTROLLER_DEBUG_HPP

#include <Arduino.h>

// #define DEBUG

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

#ifndef debug_begin
#define debug_begin(baudrate)
#endif

#ifndef debug_setup
#define debug_setup(baudrate)
#endif

#endif

inline void serial_setup(unsigned long baudrate)
{
  debug_begin(115200);   // Init serial port and set baudrate
  while (!Serial)
  {
  }
}

#endif   // DRAWER_CONTROLLER_DEBUG_HPP
