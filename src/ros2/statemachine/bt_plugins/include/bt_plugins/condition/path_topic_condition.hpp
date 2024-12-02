#ifndef DRAWER_STATEMACHINE_PATH_TOPIC_CONDITION_HPP_
#define DRAWER_STATEMACHINE_PATH_TOPIC_CONDITION_HPP_

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/condition_node.h"
#include "bt_plugins/utils/time_conversions.hpp"
#include "nav_msgs/msg/path.hpp"

namespace statemachine
{
  constexpr uint16_t DEFAULT_VALID_PATH_AGE_IN_MS = 2000;

  class PathTopicCondition : public BT::ConditionNode
  {
   public:
    PathTopicCondition(const std::string &name, const BT::NodeConfig &config);
    PathTopicCondition() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<std::string>("topic", "/path"),
              BT::InputPort<uint16_t>("valid_path_age_in_ms", std::to_string(DEFAULT_VALID_PATH_AGE_IN_MS))};
    }

   private:
    rclcpp::Node::SharedPtr _node;
    BT::Blackboard::Ptr _blackboard;

    rclcpp::CallbackGroup::SharedPtr _callback_group;
    rclcpp::executors::SingleThreadedExecutor _callback_group_executor;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_sub;

    std::string _topic_name;
    std::chrono::milliseconds _valid_path_age_in_ms;

    builtin_interfaces::msg::Time _last_path_timestamp;

    bool _received_path_once_within_valid_age = false;

    void _callback_path(const nav_msgs::msg::Path::SharedPtr msg);
  };

}   // namespace statemachine

#endif   // DRAWER_STATEMACHINE_PATH_TOPIC_CONDITION_HPP_