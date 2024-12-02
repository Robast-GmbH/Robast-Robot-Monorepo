#include "bt_plugins/condition/path_topic_condition.hpp"

namespace statemachine
{
  PathTopicCondition::PathTopicCondition(const std::string &name, const BT::NodeConfig &config)
      : BT::ConditionNode(name, config)
  {
    _node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    _blackboard = config.blackboard;
    _callback_group = _node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    _callback_group_executor.add_callback_group(_callback_group, _node->get_node_base_interface());

    uint16_t valid_path_age_in_ms = 0;
    getInput("topic", _topic_name);
    getInput("valid_path_age_in_ms", valid_path_age_in_ms);

    if (_topic_name == "")
    {
      _topic_name = "/path";
    }

    if (valid_path_age_in_ms == 0)
    {
      valid_path_age_in_ms = DEFAULT_VALID_PATH_AGE_IN_MS;
    }
    _valid_path_age_in_ms = std::chrono::milliseconds(valid_path_age_in_ms);

    _last_path_timestamp = _node->now();

    rclcpp::QoS qos_path_msgs = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
    qos_path_msgs.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_path_msgs.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos_path_msgs.avoid_ros_namespace_conventions(false);

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = _callback_group;
    _path_sub = _node->create_subscription<nav_msgs::msg::Path>(
      _topic_name,
      qos_path_msgs,
      std::bind(&PathTopicCondition::_callback_path, this, std::placeholders::_1),
      sub_option);
  }

  BT::NodeStatus PathTopicCondition::tick()
  {
    _callback_group_executor.spin_some();

    const builtin_interfaces::msg::Time current_time = _node->now();

    const std::chrono::milliseconds time_since_last_path_in_ms =
      utils::convert_to_milliseconds(current_time) - utils::convert_to_milliseconds(_last_path_timestamp);

    if (time_since_last_path_in_ms < _valid_path_age_in_ms)
    {
      // Makes sure that path is only considered once for the condition
      if (_received_path_once_within_valid_age)
      {
        RCLCPP_INFO(rclcpp::get_logger("PathTopicCondition"),
                    "Path already received once within valid age of %ld ms",
                    _valid_path_age_in_ms.count());
        return BT::NodeStatus::FAILURE;
      }

      RCLCPP_INFO(rclcpp::get_logger("PathTopicCondition"),
                  "Received path within %ld, which is within valid age of %ld ms",
                  time_since_last_path_in_ms.count(),
                  _valid_path_age_in_ms.count());
      _received_path_once_within_valid_age = true;
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      RCLCPP_DEBUG(
        rclcpp::get_logger("PathTopicCondition"), "Path is outdated by %ld ms", time_since_last_path_in_ms.count());
      _received_path_once_within_valid_age = false;
      return BT::NodeStatus::FAILURE;
    }
  }

  void PathTopicCondition::_callback_path(const nav_msgs::msg::Path::SharedPtr msg)
  {
    RCLCPP_INFO(rclcpp::get_logger("PathTopicCondition"), "Path received");
    _last_path_timestamp = msg->header.stamp;
  }
}   // namespace statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<statemachine::PathTopicCondition>("PathTopicCondition");
}
