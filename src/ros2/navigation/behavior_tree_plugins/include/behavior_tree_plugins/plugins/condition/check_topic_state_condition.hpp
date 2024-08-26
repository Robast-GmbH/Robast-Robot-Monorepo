#ifndef BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_TOPIC_STATE_CONDITION_HPP_
#define BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_TOPIC_STATE_CONDITION_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace nav2_behavior_tree
{

  class CheckTopicStateCondition : public BT::ConditionNode
  {
   public:
    CheckTopicStateCondition(const std::string& condition_name, const BT::NodeConfiguration& conf);

    CheckTopicStateCondition() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<std::string>(
                "topic", "default", "name of the topic, whose data field is checked for target state"),
              BT::InputPort<bool>("target_state", true, "target state the data field value is checked for")};
    }

   private:
    rclcpp::Node::SharedPtr _node;

    rclcpp::CallbackGroup::SharedPtr _callback_group;
    rclcpp::executors::SingleThreadedExecutor _callback_group_executor;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _subscriber;
    bool _current_state;
    bool _received_state;
    bool _target_state;
    std::string _topic_name;

    void topic_callback(const std_msgs::msg::Bool::SharedPtr msg);
  };

}   // namespace nav2_behavior_tree

#endif   // BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_TOPIC_STATE_CONDITION_HPP_