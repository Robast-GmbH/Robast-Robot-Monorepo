#include <cstdint>   // For uint8_t
#include <memory>
#include <vector>

#include "communication_interfaces/msg/drawer_address.hpp"
#include "communication_interfaces/msg/led.hpp"
#include "communication_interfaces/msg/led_cmd.hpp"

namespace bt_base_types
{
  struct LED
  {
    uint8_t red = 0;
    uint8_t blue = 0;
    uint8_t green = 0;
    uint8_t brightness = 0;

    LED &operator=(const communication_interfaces::msg::Led &ros_led)
    {
      red = ros_led.red;
      blue = ros_led.blue;
      green = ros_led.green;
      brightness = ros_led.brightness;
      return *this;
    }

    bool operator==(const communication_interfaces::msg::Led &ros_led) const
    {
      return red == ros_led.red && blue == ros_led.blue && green == ros_led.green && brightness == ros_led.brightness;
    }

    bool operator!=(const communication_interfaces::msg::Led &ros_led) const
    {
      return !(*this == ros_led);
    }

    bool operator==(const LED &led) const
    {
      return red == led.red && blue == led.blue && green == led.green && brightness == led.brightness;
    }
  };

  class Base_LED_modes
  {
   public:
    Base_LED_modes()
    {
    }

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
    static void set_percentage_leds(std::vector<LED> &leds,
                                    const LED &color,
                                    double lower_percentage,
                                    double upper_percentage)
    {
      if (lower_percentage < 0.0 || lower_percentage >= upper_percentage || upper_percentage > 100.0)
      {
        return;
      }

      size_t start_index = static_cast<size_t>(leds.size() * (lower_percentage / 100.0));
      size_t end_index = static_cast<size_t>(leds.size() * (upper_percentage / 100.0));

      for (size_t i = start_index; i < end_index; ++i)
      {
        leds[i].red = color.red;
        leds[i].green = color.green;
        leds[i].blue = color.blue;
        leds[i].brightness = color.brightness;
      }
    }

    static communication_interfaces::msg::Led to_ros_led(const LED &led)
    {
      communication_interfaces::msg::Led ros_led;
      ros_led.red = led.red;
      ros_led.blue = led.blue;
      ros_led.green = led.green;
      ros_led.brightness = led.brightness;
      return ros_led;
    }

    static communication_interfaces::msg::LedCmd to_ros_led_cmd(
        const std::vector<LED> &leds,
        const communication_interfaces::msg::DrawerAddress &drawer_address,
        const uint16_t fade_in_ms,
        const uint16_t start_index = 0)
    {
      communication_interfaces::msg::LedCmd led_cmd;
      std::vector<communication_interfaces::msg::Led> led_vector;
      led_vector.reserve(leds.size());
      for (size_t i = 0; i < leds.size(); ++i)
      {
        led_vector.push_back(to_ros_led(leds[i]));
      }
      led_cmd.leds = led_vector;
      led_cmd.drawer_address = drawer_address;
      led_cmd.fade_time_in_ms = fade_in_ms;
      led_cmd.start_index = start_index;
      return led_cmd;
    }
  };
}   // namespace bt_base_types
