#include "bt_plugins/condition/check_safety_register_condition.hpp"

namespace statemachine
{
  CheckSafetyRegisterCondition::CheckSafetyRegisterCondition(
      const std::string &name,
      const BT::NodeConfig &config) : BT::ConditionNode(name, config)
  {
    _node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    _callback_group = _node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive,
        false);
    callback_group_executor_.add_callback_group(_callback_group, _node->get_node_base_interface());

    getInput("topic", _topic_name);

    if (_topic_name == "")
    {
      auto var = getInput<std::string>("topic");
      _topic_name = "/robot/safety_module/raw_registers";
    }

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local().reliable();

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = _callback_group;
    _safety_register_sub = _node->create_subscription<robotnik_safety_msgs::msg::RegisterArray>(
        _topic_name,
        qos,
        std::bind(&CheckSafetyRegisterCondition::callbackSafetyRegisterFeedback, this, std::placeholders::_1),
        sub_option);
  }

  BT::NodeStatus CheckSafetyRegisterCondition::tick()
  {
    callback_group_executor_.spin_some();

    if (_safety_register)
    {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void CheckSafetyRegisterCondition::callbackSafetyRegisterFeedback(const robotnik_safety_msgs::msg::RegisterArray::SharedPtr msg)
  {
    for (auto register_ : msg->registers)
    {
      if (register_.key == getInput<std::string>("key"))
      {
        _safety_register = (register_.value == "True");
      }
    }
  }
} // namespace statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<statemachine::CheckSafetyRegisterCondition>("CheckSafetyRegister");
}