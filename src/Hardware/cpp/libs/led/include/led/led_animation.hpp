#ifndef LED_LED_ANIMATION_HPP
#define LED_LED_ANIMATION_HPP

#include <vector>

#include "led/led_state.hpp"

namespace led
{
  struct LedAnimation
  {
    std::vector<LedState> target_led_states;
    uint8_t fade_time_in_hundreds_of_ms;  // this will be set by the LED_HEADER CAN message
    uint16_t num_of_led_states_to_change; // this will be set by the LED_HEADER CAN message
    uint16_t start_index_led_states;      // this will be set by the LED_HEADER CAN message

    // Default Constructor
    LedAnimation()
        : target_led_states({}),
          fade_time_in_hundreds_of_ms(0),
          num_of_led_states_to_change(0),
          start_index_led_states(0) // Initialize members to default values
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
    LedAnimation &operator=(const LedAnimation &other)
    {
      if (this == &other)
      {
        return *this; // Handle self-assignment
      }

      // Copy all the non-vector members
      fade_time_in_hundreds_of_ms = other.fade_time_in_hundreds_of_ms;
      num_of_led_states_to_change = other.num_of_led_states_to_change;
      start_index_led_states = other.start_index_led_states;

      // Deep copy the vector member 'target_led_states'
      target_led_states.clear(); // Clear existing data
      target_led_states.reserve(other.target_led_states.size());
      for (const LedState &state : other.target_led_states)
      {
        target_led_states.push_back(state); // You can create a deep copy of LedState if needed
      }

      return *this;
    }
  };
} // namespace led

#endif // LED_LED_ANIMATION_HPP
