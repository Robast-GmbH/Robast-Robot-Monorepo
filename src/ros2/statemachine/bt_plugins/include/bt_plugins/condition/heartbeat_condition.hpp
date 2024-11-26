#ifndef BT_PLUGINS_CONDITION__HEARTBEAT_CONDITION_HPP
#define BT_PLUGINS_CONDITION__HEARTBEAT_CONDITION_HPP

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/condition_node.h"
#include "builtin_interfaces/msg/time.hpp"
#include "communication_interfaces/msg/heartbeat.hpp"

namespace statemachine
{
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
              BT::OutputPort<std::string>("id", "1")};
    }

    static std::chrono::milliseconds convert_to_milliseconds(const builtin_interfaces::msg::Time &time);

   private:
    rclcpp::Node::SharedPtr _node;
    BT::Blackboard::Ptr _blackboard;

    rclcpp::CallbackGroup::SharedPtr _callback_group;
    rclcpp::executors::SingleThreadedExecutor _callback_group_executor;
    rclcpp::Subscription<communication_interfaces::msg::Heartbeat>::SharedPtr _heartbeat_sub;

    std::string _topic_name;
    std::string _id;
    uint8_t _timeouts_until_failure;
    uint16_t _latency_tolerance_in_ms;
    uint16_t _heartbeat_interval_in_ms;
    builtin_interfaces::msg::Time _last_heartbeat_timestamp;

    bool _first_heartbeat_received = false;

    void _callback_heartbeat(const communication_interfaces::msg::Heartbeat::SharedPtr msg);
  };

}   // namespace statemachine

#endif   // BT_PLUGINS_CONDITION__HEARTBEAT_CONDITION_HPP