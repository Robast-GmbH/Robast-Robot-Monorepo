#ifndef LED_LED_HEADER_HPP
#define LED_LED_HEADER_HPP

namespace led
{
  struct LedHeader
  {
    const uint16_t num_of_led_states_to_change;
    const uint16_t start_index_of_leds_to_change;
    const uint8_t fade_time_in_hundreds_of_ms;

    // Default Constructor
    LedHeader() : num_of_led_states_to_change(0), start_index_of_leds_to_change(0), fade_time_in_hundreds_of_ms(0)
    {
    }

    // Parameterized Constructor
    LedHeader(const uint16_t num_of_led_states_to_change_input,
              const uint16_t start_index_of_leds_to_change_input,
              const uint8_t fade_time_in_hundreds_of_ms_input)
        : num_of_led_states_to_change(num_of_led_states_to_change_input),
          start_index_of_leds_to_change(start_index_of_leds_to_change_input),
          fade_time_in_hundreds_of_ms(fade_time_in_hundreds_of_ms_input)
    {
    }
  };
} // namespace led

#endif // LED_LED_HEADER_HPP
