#ifndef DRAWER_CONTROLLER_TIMER_HPP
#define DRAWER_CONTROLLER_TIMER_HPP

#include <Arduino.h>

#define FADING_TIMER_MINIMAL_ALARM_VALUE 390.625
#define TIMER_FACTOR                     1 * 8   // this should be an integer multiple of 8
#define FADING_TIMER_PRESCALER_VALUE     80
#define TIMER_ID                         0   // can be 0 or 1

namespace timer
{
  extern volatile uint16_t fade_counter;
  extern float max_fade_counter;
  extern hw_timer_t *fading_timer;
  extern portMUX_TYPE fading_timer_mux;

  static void IRAM_ATTR on_timer_for_fading();

  void initialize_timer();

  void disable_timer();

  void enable_timer();

  void set_max_counter_value(uint8_t new_fade_time_in_hundreds_of_ms, uint8_t factor);

  float get_max_fade_counter_value();

  float get_fade_counter_value();

}   // namespace timer

#endif   // DRAWER_CONTROLLER_TIMER_HPP