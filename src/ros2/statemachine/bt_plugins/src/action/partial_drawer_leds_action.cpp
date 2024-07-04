#include "bt_plugins/action/partial_drawer_leds_action.hpp"
#include <cassert>
#include <charconv>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string_view>
#include <system_error>

namespace BT
{
    template <>
    uint8_t convertFromString<uint8_t>(StringView str)
    {
        uint8_t result = 0;
        auto [ptr, ec] = std::from_chars(str.data(), str.data() + str.size(), result);
        if (ec != std::errc())
        {
            throw RuntimeError(StrCat("Can't convert string [", str, "] to uint8_t"));
        }
        return result;
    }
}

namespace statemachine
{
    PartialDrawerLED::PartialDrawerLED(
        const std::string &name,
        const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config)
    {
        _node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        _blackboard = config.blackboard;
        uint8_t drawer_leds_bot = 0;
        uint8_t drawer_leds_mid = 0;
        uint8_t drawer_leds_top = 0;
        getInput("topic", topic_name_);
        getInput("led_brightness_bot", drawer_leds_bot);
        getInput("led_brightness_mid", drawer_leds_mid);
        getInput("led_brightness_top", drawer_leds_top);
        communication_interfaces::msg::DrawerAddress drawer_address;
        getInput("drawer_address", drawer_address);
        _tray_leds = communication_interfaces::msg::TrayTask();
        _tray_leds.drawer_address = drawer_address;
        _tray_leds.led_brightness = {drawer_leds_bot, drawer_leds_mid, drawer_leds_top};

        initializePublisher();
    }

    void PartialDrawerLED::initializePublisher()
    {
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local().reliable();
        _tray_led_publisher = _node->create_publisher<communication_interfaces::msg::TrayTask>(topic_name_, qos);
    }

    BT::NodeStatus PartialDrawerLED::tick()
    {
        _tray_led_publisher->publish(_tray_leds);

        return BT::NodeStatus::SUCCESS;
    }

} // namespace statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<statemachine::PartialDrawerLED>("PartialDrawerLED");
}