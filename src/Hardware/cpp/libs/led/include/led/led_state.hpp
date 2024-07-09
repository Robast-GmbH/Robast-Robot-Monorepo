#ifndef DRAWER_CONTROLLER_LED_STATE_HPP
#define DRAWER_CONTROLLER_LED_STATE_HPP

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
} // namespace drawer_controller

#endif // DRAWER_CONTROLLER_LED_STATE_HPP
