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
        if (ec != std::errc())
        {
            throw RuntimeError(StrCat("Can't convert string [", str, "] to uint8_t"));
        }
        return result;
    }
}

namespace statemachine
{

    using std::placeholders::_1;

    ChangeLED::ChangeLED(
        const std::string &name,
        const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config)
    {
        _node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        blackboard_ = config.blackboard;
        getInput("led_topic", topic_name_);
        getInput("blue", drawer_leds_.blue);
        getInput("red", drawer_leds_.red);
        getInput("green", drawer_leds_.green);
        getInput("brightness", drawer_leds_.brightness);
        getInput("mode", drawer_leds_.mode);
        initializePublisher();
    }

    void ChangeLED::initializePublisher()
    {
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local().reliable();
        led_publisher_ = _node->create_publisher<communication_interfaces::msg::DrawerLeds>(topic_name_, qos);
    }

    const communication_interfaces::msg::DrawerLeds ChangeLED::getDrawerLED()
    {
        return drawer_leds_;
    }

    BT::NodeStatus ChangeLED::tick()
    {
        bool use_blackboard_address = false;
        getInput("use_blackboard_address", use_blackboard_address);
        if (!use_blackboard_address)
        {
            getInput("drawer_address", drawer_leds_.drawer_address);
        }
        else
        {
            drawer_leds_.drawer_address = blackboard_->get<communication_interfaces::msg::DrawerAddress>("drawer_address");
        }
        getInput("blue", drawer_leds_.blue);
        getInput("red", drawer_leds_.red);
        getInput("green", drawer_leds_.green);
        getInput("brightness", drawer_leds_.brightness);
        getInput("mode", drawer_leds_.mode);
        ChangeLED::publish();
        return BT::NodeStatus::SUCCESS;
    }

    void ChangeLED::publish()
    {
        led_publisher_->publish(drawer_leds_);
    }

} // namespace statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<statemachine::ChangeLED>("ChangeLED");
}