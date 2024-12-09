#include "bt_plugins/action/publish_led_action.hpp"

namespace statemachine
{

  LEDPublisherAction::LEDPublisherAction(const std::string &name, const BT::NodeConfig &config)
      : BT::SyncActionNode(name, config)
  {
    _blackboard = config.blackboard;

    getInput("led_topic", topic_name_);
    // std::scoped_lock l(blackboard_->entryMutex());
    _node = _blackboard->get<rclcpp::Node::SharedPtr>("node");
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local().reliable();

    _led_publisher = _node->create_publisher<communication_interfaces::msg::LedCmd>(topic_name_, qos);
  }

  BT::NodeStatus LEDPublisherAction::tick()
  {
    std::vector<bt_base_types::LED> led_vector;
    communication_interfaces::msg::DrawerAddress drawer_address;
    uint16_t fade_in_ms;
    bool ack_requested = false;
    int tmp = 0;
    getInput("fading_time_ms", tmp);
    fade_in_ms = (uint16_t) tmp;
    getInput("leds", led_vector);
    getInput("drawer_address", drawer_address);
    getInput("ack_requested", ack_requested);
    auto msg = bt_base_types::Base_LED_modes::to_ros_led_cmd(led_vector, drawer_address, fade_in_ms, ack_requested);
    _led_publisher->publish(msg);

    return BT::NodeStatus::SUCCESS;
  }
}   // namespace statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<statemachine::LEDPublisherAction>("LEDPublisherAction");
}