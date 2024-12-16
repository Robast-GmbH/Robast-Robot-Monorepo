#include "behavior_tree_plugins/plugins/condition/string_topic_condition.hpp"

namespace nav2_behavior_tree
{
  StringTopicCondition::StringTopicCondition(const std::string &condition_name, const BT::NodeConfiguration &conf)
      : BT::ConditionNode(condition_name, conf)
  {
    _node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    _callback_group = _node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    _callback_group_executor.add_callback_group(_callback_group, _node->get_node_base_interface());

    _received_value = false;

    getInput("topic", _topic_name);
    getInput("target_value", _target_value);

    if (_topic_name.empty())
    {
      _topic_name = "/robot/safety_module/safety_stop";
    }

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = _callback_group;
    _subscriber = _node->create_subscription<std_msgs::msg::String>(
      _topic_name, 10, std::bind(&StringTopicCondition::topic_callback, this, std::placeholders::_1), sub_option);
  }

  BT::NodeStatus StringTopicCondition::tick()
  {
    _callback_group_executor.spin_some();

    if (_received_value)
    {
      _received_value = false;
      return (_current_value == _target_value) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    else
    {
      return BT::NodeStatus::RUNNING;
    }
  }

  void StringTopicCondition::topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    _current_value = msg->data;
    _received_value = true;
  }

}   // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::StringTopicCondition>("StringTopicCondition");
}
