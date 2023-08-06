#include "bt_plugins/action/partial_colorize_leds_action.hpp"

namespace bt_plugins
{
    BT::NodeStatus PartialColorizeLED::tick()
    {
        base_types::LED led_color;
        double lower_percentage = 0.0;
        double upper_percentage = 0.0;
        std::vector<LED> leds;
        getInput("LEDs", leds);
        getInput("blue", led.blue);
        getInput("red", led.red);
        getInput("green", led.green);
        getInput("brightness", led.brightness);
        getInput("lower_bound", lower_percentage);
        getInput("upper_bound", upper_percentage);
        Base_LED_modes::set_percentage_leds(leds, led_color, lower_percentage, upper_percentage);
        setOutput("LEDs_colored", leds);
        return BT::NodeStatus::SUCCESS;
    }
} // namespace bt_plugins

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<bt_plugins::PartialColorizeLED>("PartialColorizeLED");
}
