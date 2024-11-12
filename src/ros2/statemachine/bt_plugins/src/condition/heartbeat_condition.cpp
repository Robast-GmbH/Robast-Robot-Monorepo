#include "bt_plugins/condition/heartbeat_condition.hpp"

namespace statemachine
{
  HeartbeatCondition::HeartbeatCondition(const std::string &name, const BT::NodeConfig &config)
      : BT::ConditionNode(name, config)
  {
    _node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    _blackboard = config.blackboard;
    _callback_group = _node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    _callback_group_executor.add_callback_group(_callback_group, _node->get_node_base_interface());

    getInput("timeouts_until_failure", _timeouts_until_failure);
    getInput("topic", _topic_name);
    _id = _blackboard->get<std::string>("id");

    if (_topic_name == "")
    {
      auto var = getInput<std::string>("topic");
      _topic_name = "/heartbeat";
    }

    rclcpp::QoS qos_heartbeat_msgs = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
    qos_heartbeat_msgs.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_heartbeat_msgs.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos_heartbeat_msgs.avoid_ros_namespace_conventions(false);

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = _callback_group;
    _heartbeat_sub = _node->create_subscription<communication_interfaces::msg::Heartbeat>(
        _topic_name,
        qos_heartbeat_msgs,
        std::bind(&HeartbeatCondition::_callback_heartbeat, this, std::placeholders::_1),
        sub_option);
  }

  BT::NodeStatus HeartbeatCondition::tick()
  {
    _callback_group_executor.spin_some();

    const builtin_interfaces::msg::Time current_timestamp = _node->now();

    // Convert builtin_interfaces::msg::Time to rclcpp::Time
    rclcpp::Time rcl_current_timestamp(current_timestamp);
    rclcpp::Time rcl_last_heartbeat_timestamp(_last_heartbeat_timestamp);

    const rclcpp::Duration time_since_last_heartbeat = rcl_current_timestamp - rcl_last_heartbeat_timestamp;

    std::chrono::nanoseconds failure_timeout_duration_in_ns(_heartbeat_interval_in_ms * 1000000 *
                                                            _timeouts_until_failure);

    rclcpp::Duration failure_timeout_duration(failure_timeout_duration_in_ns);

    if (time_since_last_heartbeat > failure_timeout_duration)
    {
      setOutput("id", _id);
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }

  void HeartbeatCondition::_callback_heartbeat(const communication_interfaces::msg::Heartbeat::SharedPtr msg)
  {
    if (msg->id == _id)
    {
      _heartbeat_interval_in_ms = msg->interval_in_ms;
      _last_heartbeat_timestamp = msg->stamp;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("HeartbeatCondition"), "Received heartbeat from %s. I am %s", msg->id.c_str(), _id.c_str());
  }

}   // namespace statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<statemachine::HeartbeatCondition>("HeartbeatCondition");
}
