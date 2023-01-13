#include "bt_plugins/action/change_led_action.hpp"
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
        if(ec != std::errc())
        {
            throw RuntimeError(StrCat("Can't convert string [", str, "] to uint8_t"));
        }
    return result;
    }
}

namespace drawer_statemachine
{

    using std::placeholders::_1;

    ChangeLED::ChangeLED(
        const std::string& name,
        const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

        getInput("led_topic", topic_name_);

        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local().reliable();

        rclcpp::SubscriptionOptions sub_option;
        led_publisher_ = node_->create_publisher<communication_interfaces::msg::DrawerLeds>(topic_name_, qos);
    }

    BT::NodeStatus ChangeLED::tick()
    {
        getInput("drawer_address", drawer_leds_.drawer_address);
        getInput("blue", drawer_leds_.blue);
        getInput("red", drawer_leds_.red);
        getInput("green", drawer_leds_.green);
        getInput("brightness", drawer_leds_.brightness);
        getInput("mode", drawer_leds_.mode);
        led_publisher_->publish(drawer_leds_);
        
        return BT::NodeStatus::SUCCESS;
    }

}  // namespace drawer_statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<drawer_statemachine::ChangeLED>("ChangeLED");
}