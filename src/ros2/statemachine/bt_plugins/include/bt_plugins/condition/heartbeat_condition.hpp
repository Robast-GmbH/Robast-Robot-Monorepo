#ifndef BT_PLUGINS_CONDITION__HEARTBEAT_CONDITION_HPP
#define BT_PLUGINS_CONDITION__HEARTBEAT_CONDITION_HPP

#include <rclcpp/rclcpp.hpp>
#include <unordered_map>

#include "behaviortree_cpp/condition_node.h"
#include "bt_plugins/utils/time_conversions.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "communication_interfaces/msg/heartbeat.hpp"
#include "std_msgs/msg/string.hpp"

namespace statemachine
{
  struct heartbeat_info
  {
    builtin_interfaces::msg::Time timestamp;
    uint16_t interval_in_ms;
    bool printed_single_timeout_warning;
  };

  class HeartbeatCondition : public BT::ConditionNode
  {
   public:
    HeartbeatCondition(const std::string &name, const BT::NodeConfig &config);
    HeartbeatCondition() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<std::string>("topic", "/heartbeat"),
              BT::InputPort<uint8_t>("timeouts_until_failure", "3"),
              BT::InputPort<uint16_t>("latency_tolerance_in_ms", "100"),
              BT::OutputPort<std::string>("failed_heartbeat_id", "1")};
    }

   private:
    rclcpp::Node::SharedPtr _node;
    BT::Blackboard::Ptr _blackboard;

    rclcpp::CallbackGroup::SharedPtr _callback_group;
    rclcpp::executors::SingleThreadedExecutor _callback_group_executor;
    rclcpp::Subscription<communication_interfaces::msg::Heartbeat>::SharedPtr _heartbeat_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _living_devices_pub;

    std::string _topic_name;
    uint8_t _timeouts_until_failure;
    uint16_t _latency_tolerance_in_ms;

    std::unordered_map<std::string, heartbeat_info> _heartbeat_info_by_id;

    std::unordered_set<std::string> _living_devices;

    bool _first_heartbeat_received = false;

    void _callback_heartbeat(const communication_interfaces::msg::Heartbeat::SharedPtr msg);

    void publish_living_devices();
  };

}   // namespace statemachine

#endif   // BT_PLUGINS_CONDITION__HEARTBEAT_CONDITION_HPP