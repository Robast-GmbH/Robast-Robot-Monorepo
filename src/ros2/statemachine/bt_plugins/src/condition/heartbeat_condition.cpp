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
    getInput("latency_tolerance_in_ms", _latency_tolerance_in_ms);

    if (_topic_name == "")
    {
      _topic_name = "/heartbeat";
    }

    rclcpp::QoS qos_heartbeat_msgs = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
    qos_heartbeat_msgs.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_heartbeat_msgs.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos_heartbeat_msgs.avoid_ros_namespace_conventions(false);

    rclcpp::QoS qos_living_devices = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
    qos_living_devices.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_living_devices.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos_living_devices.avoid_ros_namespace_conventions(false);

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = _callback_group;
    _heartbeat_sub = _node->create_subscription<communication_interfaces::msg::Heartbeat>(
        _topic_name,
        qos_heartbeat_msgs,
        std::bind(&HeartbeatCondition::_callback_heartbeat, this, std::placeholders::_1),
        sub_option);

    _living_devices_pub = _node->create_publisher<std_msgs::msg::String>("/living_devices", qos_living_devices);
  }

  BT::NodeStatus HeartbeatCondition::tick()
  {
    _callback_group_executor.spin_some();

    if (!_first_heartbeat_received)
    {
      RCLCPP_DEBUG(rclcpp::get_logger("HeartbeatCondition"), "Waiting for first heartbeat");
      return BT::NodeStatus::RUNNING;
    }

    // Use an iterator-based loop to safely erase elements
    for (auto it = _last_heartbeat_timestamp_by_id.begin(); it != _last_heartbeat_timestamp_by_id.end();)
    {
      const std::string &id = it->first;
      const rclcpp::Time &last_timestamp = it->second;
      const std::chrono::milliseconds time_since_last_heartbeat_in_ms =
          utils::convert_to_milliseconds(_node->now()) - utils::convert_to_milliseconds(last_timestamp);
      const std::chrono::milliseconds failure_timeout_in_ms(_heartbeat_interval_in_ms_by_id[id] *
                                                            _timeouts_until_failure);

      if (time_since_last_heartbeat_in_ms > failure_timeout_in_ms)
      {
        RCLCPP_ERROR(
            rclcpp::get_logger("HeartbeatCondition"),
            "HeartbeatCondition FAILURE. Timeout for id %s occurred! Last heartbeat was %ld ms ago. Timeout is %ld ms.",
            id.c_str(),
            time_since_last_heartbeat_in_ms.count(),
            failure_timeout_in_ms.count());

        _living_devices.erase(id);

        setOutput("failed_heartbeat_id", id);

        publish_living_devices();

        // Safely erase the element and update the iterator
        it = _last_heartbeat_timestamp_by_id.erase(it);

        return BT::NodeStatus::FAILURE;
      }
      else
      {
        const std::chrono::milliseconds acceptable_heartbeat_delay_in_ms(_heartbeat_interval_in_ms_by_id[id] +
                                                                         _latency_tolerance_in_ms);
        if (!_printed_single_timeout_warning[id] && time_since_last_heartbeat_in_ms > acceptable_heartbeat_delay_in_ms)
        {
          RCLCPP_WARN(rclcpp::get_logger("HeartbeatCondition"),
                      "HeartbeatCondition WARNING. Single timeout for id %s occurred! Last heartbeat was %ld ms ago. "
                      "Acceptable "
                      "delay is %ld ms.",
                      id.c_str(),
                      time_since_last_heartbeat_in_ms.count(),
                      acceptable_heartbeat_delay_in_ms.count());
          _printed_single_timeout_warning[id] = true;
        }

        // Move to the next element if no deletion occurred
        ++it;
      }
    }

    return BT::NodeStatus::RUNNING;
  }

  void HeartbeatCondition::publish_living_devices()
  {
    std_msgs::msg::String msg;
    msg.data = "";

    uint16_t count = 0;
    // Copy _living_devices to a vector and sort it
    std::vector<std::string> devices(_living_devices.begin(), _living_devices.end());
    std::sort(devices.begin(), devices.end());

    for (const auto &device : devices)
    {
      if (count != 0 && count != devices.size())
      {
        msg.data += ",";
      }
      msg.data += device;
      count++;
    }

    _living_devices_pub->publish(msg);
  }

  void HeartbeatCondition::_callback_heartbeat(const communication_interfaces::msg::Heartbeat::SharedPtr msg)
  {
    _last_heartbeat_timestamp_by_id[msg->id] = msg->stamp;
    _printed_single_timeout_warning[msg->id] = false;
    _heartbeat_interval_in_ms_by_id[msg->id] = msg->interval_in_ms;
    _first_heartbeat_received = true;

    RCLCPP_DEBUG(rclcpp::get_logger("HeartbeatCondition"),
                 "Received heartbeat from %s. My interval is %d ms and the "
                 "last heartbeat was at %ld ms.",
                 msg->id.c_str(),
                 _heartbeat_interval_in_ms_by_id[msg->id],
                 utils::convert_to_milliseconds(msg->stamp).count());

    // Go through all living devices and check if the new device is already in the list
    if (_living_devices.find(msg->id) == _living_devices.end())
    {
      RCLCPP_INFO(rclcpp::get_logger("HeartbeatCondition"), "Added new device %s to living devices", msg->id.c_str());
      _living_devices.insert(msg->id);
      publish_living_devices();
    }
  }

}   // namespace statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<statemachine::HeartbeatCondition>("HeartbeatCondition");
}
