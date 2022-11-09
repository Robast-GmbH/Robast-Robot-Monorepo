#include "communication_interfaces/msg/drawer_leds.hpp"
namespace led_color
{
    /// @brief struct to be used for led description. Correlating to "DrawerLeds.msg"
    struct led_color
    {
        uint8_t red = 0;
        uint8_t blue = 0;
        uint8_t green = 0;
        uint8_t brightness = 0;
        uint8_t mode = 0;
    };


    /// @brief translate the "led_color" struct to the DrawerLeds.msg.
    /// @param led_color 
    /// @return DrawerLeds.msg without Drawer Address
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