#include "bt_plugins/action/partial_colorize_leds_action.hpp"

namespace bt_plugins
{
    PartialColorizeLED::PartialColorizeLED(const std::string &name, const BT::NodeConfig &config) : BT::SyncActionNode(name, config)
    {
    }
    BT::NodeStatus PartialColorizeLED::tick()
    {
        bt_base_types::LED led_color;
        double lower_percentage = 0.0;
        double upper_percentage = 0.0;
        std::vector<bt_base_types::LED> leds;
        getInput("LEDs", leds);
        getInput("blue", led_color.blue);
        getInput("red", led_color.red);
        getInput("green", led_color.green);
        getInput("brightness", led_color.brightness);
        getInput("lower_bound", lower_percentage);
        getInput("upper_bound", upper_percentage);
        bt_base_types::Base_LED_modes::set_percentage_leds(leds, led_color, lower_percentage, upper_percentage);
        setOutput("LEDs_colored", leds);
        return BT::NodeStatus::SUCCESS;
    }
} // namespace bt_plugins

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<bt_plugins::PartialColorizeLED>("PartialColorizeLED");
}
