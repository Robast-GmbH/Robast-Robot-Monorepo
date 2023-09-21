// TODO: @Tobi this is just theorycraft. We need a new docker including the robotnik msgs first.
#include "bt_plugins/condition/is_robot_charging.hpp"

namespace statemachine
{
  IsRobotCharging::IsRobotCharging(const std::string &name, const BT::NodeConfig &config) : BT::ConditionNode(name, config)
  {
    _node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    _callback_group = _node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    _callback_group_executor.add_callback_group(_callback_group, _node->get_node_base_interface());
    getInput("target_value", _target_value);
    getInput("topic", _topic_name);
    _battery_status_sub = _node->create_subscription<robotnik_msgs::msg::BatteryStatus>(
        _topic_name, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&IsRobotCharging::callbackBatteryFeedback, this, std::placeholders::_1), rmw_qos_profile_default, _callback_group);
  }

  BT::NodeStatus IsRobotCharging::tick()
  {
    callback_group_executor_.spin_some();
    RCLCPP_DEBUG(rclcpp::get_logger("IsRobotCharging"), "ticked");
    if (_last_message == _target_value)
    {
      RCLCPP_DEBUG(rclcpp::get_logger("IsRobotCharging"), "BT::NodeStatus::SUCCESS");
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      RCLCPP_DEBUG(rclcpp::get_logger("IsRobotCharging"), "BT::NodeStatus::FAILURE");
      return BT::NodeStatus::FAILURE;
    }
  }

  void IsRobotCharging::callbackBatteryFeedback(const robotnik_msgs::msg::BatteryStatus::SharedPtr msg)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("IsRobotCharging"), "callback got called ");
    _last_message = msg->is_charging;
    setOutput("battery_level", msg->level);
  }
} // namespace statemachine