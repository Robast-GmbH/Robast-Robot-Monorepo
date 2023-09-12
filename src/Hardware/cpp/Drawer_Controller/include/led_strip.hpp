#ifndef DRAWER_CONTROLLER_LED_STRIP_HPP
#define DRAWER_CONTROLLER_LED_STRIP_HPP

#include <Arduino.h>
#include <FastLED.h>

#include <optional>

#include "can/can_db.hpp"
#include "can/can_helper.h"
#include "debug.hpp"
#include "pinout_defines.hpp"
#include "timer.hpp"

#define NUM_OF_LEDS                         18
#define MAX_NUM_OF_LED_MODES_IN_QUEUE       3
#define LED_ANIMATION_APPLYING_PERIOD_IN_MS 1

#define LED_INIT_ANIMATION_FADE_TIME_IN_MS 3000
#define LED_INIT_ANIMATION_START_INDEX     0
#define LED_INIT_RED                       0
#define LED_INIT_GREEN                     155
#define LED_INIT_BLUE                      155
#define LED_INIT_BRIGHTNESS                25

namespace drawer_controller
{

  struct LedState
  {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint8_t brightness;

    // Default Constructor
    LedState() : red(0), green(0), blue(0), brightness(0)
    {
    }

    // Parameterized Constructor
    LedState(const uint8_t red_input,
             const uint8_t green_input,
             const uint8_t blue_input,
             const uint8_t brightness_input)
        : red(red_input), green(green_input), blue(blue_input), brightness(brightness_input)
    {
    }
  };

  struct LedAnimation
  {
    std::vector<LedState> target_led_states;
    uint8_t fade_time_in_hundreds_of_ms;    // this will be set by the LED_HEADER CAN message
    uint16_t num_of_led_states_to_change;   // this will be set by the LED_HEADER CAN message
    uint16_t start_index_led_states;        // this will be set by the LED_HEADER CAN message

    // Default Constructor
    LedAnimation()
        : target_led_states({}),
          fade_time_in_hundreds_of_ms(0),
          num_of_led_states_to_change(0),
          start_index_led_states(0)   // Initialize members to default values
    {
    }

    // Parameterized Constructor
    LedAnimation(std::vector<LedState> target_led_states_input,
                 const uint8_t fade_time_in_hundreds_of_ms_input,
                 const uint16_t num_of_led_states_to_change_input,
                 const uint16_t start_index_led_states_input)
        : target_led_states(target_led_states_input),
          fade_time_in_hundreds_of_ms(fade_time_in_hundreds_of_ms_input),
          num_of_led_states_to_change(num_of_led_states_to_change_input),
          start_index_led_states(start_index_led_states_input)
    {
    }

    // Deep copy assignment operator
    LedAnimation& operator=(const LedAnimation& other)
    {
      if (this == &other)
      {
        return *this;   // Handle self-assignment
      }

      // Copy all the non-vector members
      fade_time_in_hundreds_of_ms = other.fade_time_in_hundreds_of_ms;
      num_of_led_states_to_change = other.num_of_led_states_to_change;
      start_index_led_states = other.start_index_led_states;

      // Deep copy the vector member 'target_led_states'
      target_led_states.clear();   // Clear existing data
      target_led_states.reserve(other.target_led_states.size());
      for (const LedState& state : other.target_led_states)
      {
        target_led_states.push_back(state);   // You can create a deep copy of LedState if needed
      }

      return *this;
    }
  };

  class LedStrip
  {
   public:
    LedStrip();
    // TODO@Jacob: We have the same queue in can_utils. Make one class out of it
    void add_element_to_led_animation_queue(LedAnimation led_animation);

    // TODO@Jacob: We have the same queue in can_utils. Make one class out of it
    std::optional<LedAnimation> get_element_from_led_animation_queue();

    void handle_led_control();

    void initialize_led_strip();

    void initialize_led_state_change(uint16_t num_of_led_states,
                                     uint16_t start_index_led_states_input,
                                     uint8_t fade_time_in_hundreds_of_ms_input);

    void set_led_state(LedState state);

   private:
    bool _fading_in_progress;
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
    uint8_t _head_of_led_animations_queue;

    LedAnimation _new_target_led_animation;   // the new target led animation that is successive filled by can msgs

    uint16_t _current_index_led_states;

    CRGBArray<NUM_OF_LEDS> _leds;

    unsigned long _previous_millis;   // makes sure that applying led animations is not done more often then required

    void led_init_mode();

    void init_fading(uint8_t new_fade_time_in_hundreds_of_ms);

    void apply_led_states_to_led_strip();

    float linear_interpolation(float a, float b, float t);

    void set_current_led_states_to_target_led_states();

    void handle_fading();

    void initialize_queue();

    void set_num_of_leds_to_change_to_value_within_bounds(uint16_t num_of_led_states, uint16_t start_index_led_states);
  };

  /*********************************************************************************************************
   FUNCTIONS
  *********************************************************************************************************/

}   // namespace drawer_controller

#endif   // DRAWER_CONTROLLER_LED_STRIP_HPP