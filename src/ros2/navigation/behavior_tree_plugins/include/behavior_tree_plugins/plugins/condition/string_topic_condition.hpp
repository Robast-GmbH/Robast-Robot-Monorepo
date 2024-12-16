#ifndef BEHAVIOR_TREE__PLUGINS__CONDITION__STRING_TOPIC_CONDITION_HPP_
#define BEHAVIOR_TREE__PLUGINS__CONDITION__STRING_TOPIC_CONDITION_HPP_

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp_v3/condition_node.h"
#include "std_msgs/msg/string.hpp"

namespace nav2_behavior_tree
{
  class StringTopicCondition : public BT::ConditionNode
  {
   public:
    StringTopicCondition(const std::string& condition_name, const BT::NodeConfiguration& conf);

    StringTopicCondition() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<std::string>(
          "topic", "default", "name of the topic, whose data field is checked for target value"),
        BT::InputPort<std::string>("target_value", "default", "target value the data field value is checked for")};
    }

   private:
    rclcpp::Node::SharedPtr _node;
    std::string _topic_name;

    rclcpp::CallbackGroup::SharedPtr _callback_group;
    rclcpp::executors::SingleThreadedExecutor _callback_group_executor;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscriber;

    bool _received_value;
    std::string _current_value;
    std::string _target_value;

    void topic_callback(const std_msgs::msg::String::SharedPtr msg);
  };

}   // namespace nav2_behavior_tree

#endif   // BEHAVIOR_TREE__PLUGINS__CONDITION__STRING_TOPIC_CONDITION_HPP_