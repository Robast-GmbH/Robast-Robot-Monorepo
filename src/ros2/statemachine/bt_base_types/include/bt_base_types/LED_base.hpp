#include <vector>
#include <memory>
#include <cstdint> // For uint8_t

#include "communication_interfaces/msg/drawer_address.hpp"
#include "communication_interfaces/msg/drawer_leds.hpp"

namespace bt_base_types
{
  struct LED
  {
    uint8_t red = 0;
    uint8_t blue = 0;
    uint8_t green = 0;
    uint8_t brightness = 0;
  };

  class Base_LED_modes
  {
  public:
    Base_LED_modes() {}

    // could be solved by using smartpointers, but this is much more efficient
    static void set_colors(std::vector<LED> &leds, const LED &color)
    {
      for (auto &led : leds)
      {
        led.red = color.red;
        led.green = color.green;
        led.blue = color.blue;
        led.brightness = color.brightness;
      }
    }
    static void set_percentage_leds(std::vector<LED> &leds, const LED &color, double lower_percentage, double upper_percentage)
    {
      if (lower_percentage < 0.0 || lower_percentage >= upper_percentage || upper_percentage > 100.0)
      {
        return;
      }

      size_t num_leds_to_set = static_cast<size_t>(leds.size() * ((upper_percentage - lower_percentage) / 100.0));
      size_t start_index = static_cast<size_t>(leds.size() * (lower_percentage / 100.0));
      size_t end_index = start_index + num_leds_to_set;

      for (size_t i = start_index; i < end_index; ++i)
      {
        leds[i].red = color.red;
        leds[i].green = color.green;
        leds[i].blue = color.blue;
        leds[i].brightness = color.brightness;
      }
    }

    static communication_interfaces::msg::DrawerLeds to_base_leds(const LED &led)
    {
      communication_interfaces::msg::DrawerLeds drawer_led;
      drawer_led.red = led.red;
      drawer_led.blue = led.blue;
      drawer_led.green = led.green;
      drawer_led.brightness = led.brightness;
      drawer_led.mode = 1;
      drawer_led.drawer_address.module_id = 0;
      drawer_led.drawer_address.drawer_id = 0;

      return drawer_led;
    }

    static LED from_drawer_leds(const communication_interfaces::msg::DrawerLeds &drawer_led)
    {
      LED led;
      led.red = drawer_led.red;
      led.blue = drawer_led.blue;
      led.green = drawer_led.green;
      led.brightness = drawer_led.brightness;
      return led;
    }
  };
}