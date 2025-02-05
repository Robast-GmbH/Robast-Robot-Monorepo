#include "bt_plugins/condition/electric_drawer_status_condition.hpp"

namespace statemachine
{
  ElectricDrawerStatusCondition::ElectricDrawerStatusCondition(const std::string &name, const BT::NodeConfig &config)
      : BaseCompareCondition(name, config, rclcpp::QoS(rclcpp::KeepLast(1)).reliable())
  {
  }

  BT::NodeStatus ElectricDrawerStatusCondition::tick()
  {
    callback_group_executor_.spin_some();

    RCLCPP_DEBUG(rclcpp::get_logger("ElectricDrawerStatusCondition"), "ticked");

    initialize_target_value();

    if (comparator(last_message_, target_value_))
    {
      RCLCPP_DEBUG(rclcpp::get_logger("ElectricDrawerStatusCondition"), "BT::NodeStatus::SUCCESS");
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  bool ElectricDrawerStatusCondition::comparator(communication_interfaces::msg::ElectricalDrawerStatus last_message,
                                                 uint8_t target_value)
  {
    const bool within_target_range = (target_value > 250 || last_message.position <= target_value + 5) &&
                                     (target_value < 5 || last_message.position >= target_value - 5);
    const bool stall_guard_triggered = _use_stallguard && last_message.is_stall_guard_triggered;

    if ((within_target_range && new_value_received_) || stall_guard_triggered)
    {
      RCLCPP_DEBUG(rclcpp::get_logger("ElectricDrawerStatusCondition"), "value received: %d", last_message.position);
      RCLCPP_DEBUG(rclcpp::get_logger("ElectricDrawerStatusCondition"), "target value should be %d", target_value);
      RCLCPP_DEBUG(
          rclcpp::get_logger("ElectricDrawerStatusCondition"), "stallguard: %d", last_message.is_stall_guard_triggered);
      new_value_received_ = false;
      return true;
    }

    new_value_received_ = false;
    return false;
  }

  void ElectricDrawerStatusCondition::callbackTopicFeedback(
      const communication_interfaces::msg::ElectricalDrawerStatus::SharedPtr msg)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("ElectricDrawerStatusCondition"), "callback got called ");
    last_message_ = *msg;
    new_value_received_ = true;
  }

  void ElectricDrawerStatusCondition::initialize_target_value()
  {
    getInput("target_value", target_value_);
    getInput("use_stallguard", _use_stallguard);
  }
}   // namespace statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<statemachine::ElectricDrawerStatusCondition>("ElectricDrawerStatusCondition");
}