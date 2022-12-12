#include "bt_plugins/action/change_led_action.hpp"



namespace drawer_statemachine
{

    using std::placeholders::_1;

    ChangeLED::ChangeLED(
        const std::string& name,
        const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

        getInput("led_topic", topic_name_);

        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local().reliable();

        rclcpp::SubscriptionOptions sub_option;
        // sub_option.callback_group = callback_group_;
        led_publisher_ = node_->create_publisher<communication_interfaces::msg::DrawerLeds>(topic_name_, qos);
    }

    BT::NodeStatus ChangeLED::tick()
    {
        communication_interfaces::msg::DrawerLeds drawer_leds;
        getInput("drawer_address", drawer_leds.drawer_address);
        getInput("blue", drawer_leds.blue);
        getInput("red", drawer_leds.red);
        getInput("green", drawer_leds.green);
        getInput("brightness", drawer_leds.brightness);
        getInput("mode", drawer_leds.mode);
        led_publisher_->publish(drawer_leds);
        
        return BT::NodeStatus::SUCCESS;
    }

}  // namespace drawer_statemachine

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<drawer_statemachine::ChangeLED>("ChangeLED");
}