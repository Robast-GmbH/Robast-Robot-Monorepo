#ifndef LED_LED_STATE_HPP
#define LED_LED_STATE_HPP

namespace led
{
  struct LedState
  {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint8_t brightness;
    bool is_group_state;

    // Default Constructor
    LedState() : red(0), green(0), blue(0), brightness(0), is_group_state(false)
    {
    }

    // Parameterized Constructor
    LedState(const uint8_t red_input,
             const uint8_t green_input,
             const uint8_t blue_input,
             const uint8_t brightness_input,
             const bool is_group_state_input)
        : red(red_input),
          green(green_input),
          blue(blue_input),
          brightness(brightness_input),
          is_group_state(is_group_state_input)
    {
    }
  };
}   // namespace led

#endif   // LED_LED_STATE_HPP
