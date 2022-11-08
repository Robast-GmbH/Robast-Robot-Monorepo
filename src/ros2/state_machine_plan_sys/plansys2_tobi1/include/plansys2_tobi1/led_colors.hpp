#include "communication_interfaces/msg/drawer_leds.hpp"
namespace led_color
{
    struct led_color
    {
        uint8_t red;
        uint8_t blue;
        uint8_t green;
        uint8_t brightness;
        uint8_t mode;
    };

    static communication_interfaces::msg::DrawerLeds add_leds_to_msg(led_color led_color)
    {
        communication_interfaces::msg::DrawerLeds msg;
        msg.blue = led_color.blue;
        msg.red = led_color.red;
        msg.green = led_color.green;
        msg.brightness = led_color.brightness;
        msg.mode = led_color.mode;

        return msg;
    }
}