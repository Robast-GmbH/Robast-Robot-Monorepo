// TODO: @Tobi this is just theorycraft. We need a new docker including the robotnik msgs first.
#include "bt_plugins/condition/is_robot_charging_condition.hpp"

namespace statemachine
{
  IsRobotChargingCondition::IsRobotChargingCondition(const std::string &name, const BT::NodeConfig &config) : BT::ConditionNode(name, config)
  {
    _node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    rclcpp::SubscriptionOptions sub_option;
    _callback_group = _node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive,
        false);
    callback_group_executor_.add_callback_group(_callback_group, _node->get_node_base_interface());
    sub_option.callback_group = _callback_group;
    getInput("target_value", _target_value);
    getInput("topic", _topic_name);
    _battery_status_sub = _node->create_subscription<robotnik_msgs::msg::BatteryStatus>(
        _topic_name, rclcpp::QoS(rclcpp::KeepLast(2)).best_effort(), std::bind(&IsRobotChargingCondition::callbackBatteryFeedback, this, std::placeholders::_1), sub_option);
  }

  BT::NodeStatus IsRobotChargingCondition::tick()
  {
    callback_group_executor_.spin_some();
    RCLCPP_DEBUG(rclcpp::get_logger("IsRobotChargingCondition"), "ticked");
    if (_last_message == _target_value)
    {
      RCLCPP_DEBUG(rclcpp::get_logger("IsRobotChargingCondition"), "BT::NodeStatus::SUCCESS");
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      RCLCPP_DEBUG(rclcpp::get_logger("IsRobotChargingCondition"), "BT::NodeStatus::FAILURE");
      return BT::NodeStatus::FAILURE;
    }
  }

  void IsRobotChargingCondition::callbackBatteryFeedback(const robotnik_msgs::msg::BatteryStatus::SharedPtr msg)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("IsRobotChargingCondition"), "callback got called with battery level: %f", msg->level);
    _last_message = msg->is_charging;
    _battery_level = msg->level;
    setOutput("battery_level", _battery_level);
  }
} // namespace statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<statemachine::IsRobotChargingCondition>("IsRobotChargingCondition");
}