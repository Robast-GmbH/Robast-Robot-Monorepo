#include "timer/timer.hpp"

namespace timer
{
  volatile uint16_t fade_counter = 0;
  float max_fade_counter = 0;
  hw_timer_t *fading_timer = NULL;
  portMUX_TYPE fading_timer_mux = portMUX_INITIALIZER_UNLOCKED;

  static void IRAM_ATTR on_timer_for_fading()
  {
    portENTER_CRITICAL_ISR(&fading_timer_mux);
    ++fade_counter;
    portEXIT_CRITICAL_ISR(&fading_timer_mux);
  }

  void initialize_timer()
  {
    portENTER_CRITICAL_ISR(&fading_timer_mux);
    fade_counter = 0;
    portEXIT_CRITICAL_ISR(&fading_timer_mux);
    fading_timer =
      timerBegin(TIMER_ID, FADING_TIMER_PRESCALER_VALUE, true);   // frequency of 80Mhz -> prescaler 80 makes it 1Mhz
    timerAttachInterrupt(fading_timer, &on_timer_for_fading, true);

    // The minimal fade time we can set is 100ms. To be able to display this fade time in 256 (8 bit) brightness steps
    // we need the timer to trigger 256 times within 100ms.
    // Therefore we need a period of T = 100ms * 1/256 = 0.0003906250s
    // The period is determined by T = alarm_value / f
    // Therefore alarm_value = T * f = 0.0003906250s * 1Mhz = 390.625
    // Multiplying this by 8 gives us 3125
    uint64_t alarm_value = FADING_TIMER_MINIMAL_ALARM_VALUE * TIMER_FACTOR;
    timerAlarmWrite(fading_timer, alarm_value, true);
    timerAlarmEnable(fading_timer);
  }

  void disable_timer()
  {
    if (fading_timer != NULL)
    {
      timerAlarmDisable(fading_timer);   // Disable the timer alarm
      // timerEnd(fading_timer);            // Stop and free timer //TODO: remove this?
    }
  }

  void enable_timer()
  {
    if (fading_timer != NULL)
    {
      portENTER_CRITICAL_ISR(&fading_timer_mux);
      fade_counter = 0;
      portEXIT_CRITICAL_ISR(&fading_timer_mux);
      timerAlarmEnable(fading_timer);
    }
  }

  void set_max_counter_value(uint8_t new_fade_time_in_hundreds_of_ms, uint8_t factor)
  {
    // check comment in initialize_timer() function for derivation of this calculation
    max_fade_counter = new_fade_time_in_hundreds_of_ms * (UINT8_MAX / factor);
  }

  float get_max_fade_counter_value()
  {
    return max_fade_counter;
  }

  float get_fade_counter_value()
  {
    return static_cast<float>(fade_counter);
  }
}   // namespace timer