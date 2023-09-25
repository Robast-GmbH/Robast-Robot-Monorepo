#ifndef DRAWER_CONTROLLER_LED_STRIP_HPP
#define DRAWER_CONTROLLER_LED_STRIP_HPP

#include <Arduino.h>
#include <FastLED.h>

#include <algorithm>
#include <optional>

#include "can/can_db.hpp"
#include "can/can_helper.h"
#include "debug/debug.hpp"
#include "led/led_animation.hpp"
#include "led/led_header.hpp"
#include "led/led_state.hpp"
#include "peripherals/pinout_defines.hpp"
#include "timer/timer.hpp"

#define NUM_OF_LEDS                         18
#define MAX_NUM_OF_LED_MODES_IN_QUEUE       3

#define LED_INIT_ANIMATION_FADE_TIME_IN_MS 3000
#define LED_INIT_ANIMATION_START_INDEX     0
#define LED_INIT_RED                       0
#define LED_INIT_GREEN                     155
#define LED_INIT_BLUE                      155
#define LED_INIT_BRIGHTNESS                25
#define LED_MAX_BRIGHTNESS                 255

#define FULL_PROGRESS_LED_FADING 1.0

namespace drawer_controller
{
  class LedStrip
  {
   public:
    LedStrip();

    void handle_led_control();

    void initialize_led_state_change(const LedHeader led_header);

    void set_led_state(LedState state);

   private:
    bool _is_fading_in_progress = false;
    std::vector<LedState> _starting_led_states;   // used for fading from starting to target led state
    std::vector<LedState> _current_led_states;    // used to apply current led state to led strip

    LedAnimation _target_led_animation;   // the current target led animation, which is applied withing fading time

    // this queue makes sure, that a requested led modes gets its time to finish the animation before the next led
    // mode is started
    // Please mind: Normaly you would not built a queue yourself and use xQueue from FreeRTOS or std::queue
    // But: std:queue did not work and just permanently threw expections that made the ESP32 reboot
    // To use xQueue you need to use xQueueCreate to create a queue and you need to specify the size, in bytes, required
    // to hold each item in the queue, which is not possible for the CanMessage class because it contains a vector of
    // CanSignals which has a different length depending on the CanMessage.
    // Therefore we built a queue with a vector, which should be fine in this case as the queue usually only contains
    // one or two feedback messages and is rarely used. Furthermore we try to keep it as efficient as possible and try
    // to follow what is explained here: https://youtu.be/fHNmRkzxHWs?t=2541
    std::vector<LedAnimation> _led_animations_queue;
    uint8_t _head_of_led_animations_queue = 0;

    LedAnimation _new_target_led_animation;   // the new target led animation that is successively filled by can msgs

    uint16_t _current_index_led_states = 0;

    CRGBArray<NUM_OF_LEDS> _leds;

    unsigned long _previous_millis = 0;   // makes sure that applying led animations is not done more then required

    void led_init_mode();

    void init_fading(const uint8_t new_fade_time_in_hundreds_of_ms);

    void apply_led_states_to_led_strip();

    float linear_interpolation(const float a, const float b, const float t);

    void set_current_led_states_to_target_led_states();

    void handle_fading();

    void initialize_queue();

    void set_num_of_leds_to_change_to_value_within_bounds(const uint16_t num_of_led_states,
                                                          const uint16_t start_index_led_states);

    // TODO@Jacob: We have the same queue in can_utils. Make one class out of it
    std::optional<LedAnimation> get_element_from_led_animation_queue();

    // TODO@Jacob: We have the same queue in can_utils. Make one class out of it
    void add_element_to_led_animation_queue(LedAnimation led_animation);

    void initialize_led_strip();
  };

  /*********************************************************************************************************
   FUNCTIONS
  *********************************************************************************************************/

}   // namespace drawer_controller

#endif   // DRAWER_CONTROLLER_LED_STRIP_HPP