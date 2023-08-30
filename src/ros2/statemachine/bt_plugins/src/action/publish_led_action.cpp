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
    std::vector<bt_base_types::LED> led_states;
    communication_interfaces::msg::DrawerAddress drawer_address;
    uint16_t fade_in_ms;
    getInput("fading_time_ms", fade_in_ms);
    getInput("leds", led_states);
    getInput("drawer_address", drawer_address);
    // bt_base_types::Base_LED_modes::to_ros_led_states();
    // TODO @Tobi: add converter and publish to ros topic
    return BT::NodeStatus::SUCCESS;
  }
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<statemachine::LEDPublisherAction>("LEDPublisherAction");
}