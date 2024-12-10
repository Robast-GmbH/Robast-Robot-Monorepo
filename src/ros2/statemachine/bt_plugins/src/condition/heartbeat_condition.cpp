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
    _printed_single_timeout_warning = false;

    getInput("timeouts_until_failure", _timeouts_until_failure);
    getInput("topic", _topic_name);
    getInput("latency_tolerance_in_ms", _latency_tolerance_in_ms);

    if (_topic_name == "")
    {
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

    RCLCPP_INFO(_node->get_logger(),
                "HeartbeatCondition constructor finished for id %s and topic %s",
                _id.c_str(),
                _topic_name.c_str());
  }

  BT::NodeStatus HeartbeatCondition::tick()
  {
    _callback_group_executor.spin_some();

    if (!_first_heartbeat_received)
    {
      RCLCPP_DEBUG(rclcpp::get_logger("HeartbeatCondition"), "Waiting for first heartbeat of id %s", _id.c_str());
      return BT::NodeStatus::RUNNING;
    }

    const std::chrono::milliseconds time_since_last_heartbeat_in_ms =
        utils::convert_to_milliseconds(_node->now()) - utils::convert_to_milliseconds(_last_heartbeat_timestamp);
    const std::chrono::milliseconds failure_timeout_in_ms(_heartbeat_interval_in_ms * _timeouts_until_failure);

    if (time_since_last_heartbeat_in_ms > failure_timeout_in_ms)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("HeartbeatCondition"),
          "HeartbeatCondition FAILURE. Timeout for id %s occurred! Last heartbeat was %ld ms ago. Timeout is %ld ms.",
          _id.c_str(),
          time_since_last_heartbeat_in_ms.count(),
          failure_timeout_in_ms.count());

      setOutput("id", _id);
      _first_heartbeat_received = false;
      return BT::NodeStatus::FAILURE;
    }

    const std::chrono::milliseconds acceptable_heartbeat_delay_in_ms(_heartbeat_interval_in_ms +
                                                                     _latency_tolerance_in_ms);
    if (!_printed_single_timeout_warning && time_since_last_heartbeat_in_ms > acceptable_heartbeat_delay_in_ms)
    {
      RCLCPP_WARN(
          rclcpp::get_logger("HeartbeatCondition"),
          "HeartbeatCondition WARNING. Single timeout for id %s occurred! Last heartbeat was %ld ms ago. Acceptable "
          "delay is %ld ms.",
          _id.c_str(),
          time_since_last_heartbeat_in_ms.count(),
          acceptable_heartbeat_delay_in_ms.count());
      _printed_single_timeout_warning = true;
    }

    return BT::NodeStatus::SUCCESS;
  }

  void HeartbeatCondition::_callback_heartbeat(const communication_interfaces::msg::Heartbeat::SharedPtr msg)
  {
    _id = _blackboard->get<std::string>("id");

    if (msg->id == _id)
    {
      _heartbeat_interval_in_ms = msg->interval_in_ms;
      _last_heartbeat_timestamp = msg->stamp;
      _first_heartbeat_received = true;
      _printed_single_timeout_warning = false;

      RCLCPP_DEBUG(rclcpp::get_logger("HeartbeatCondition"),
                   "Received heartbeat from %s. I am %s, my interval is %d ms and the "
                   "last heartbeat was at %ld ms.",
                   msg->id.c_str(),
                   _id.c_str(),
                   _heartbeat_interval_in_ms,
                   utils::convert_to_milliseconds(_last_heartbeat_timestamp).count());
    }
  }

}   // namespace statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<statemachine::HeartbeatCondition>("HeartbeatCondition");
}
