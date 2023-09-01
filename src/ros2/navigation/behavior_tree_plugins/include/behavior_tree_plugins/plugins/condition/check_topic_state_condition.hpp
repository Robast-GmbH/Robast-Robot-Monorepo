#ifndef BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_TOPIC_STATE_CONDITION_HPP_
#define BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_TOPIC_STATE_CONDITION_HPP_

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace nav2_behavior_tree
{

  /**
   * @brief A BT::ConditionNode that returns SUCCESS when goal is
   * updated on the blackboard and FAILURE otherwise
   */
  class CheckTopicStateCondition : public BT::ConditionNode
  {
   public:
    /**
     * @brief A constructor for CheckTopicStateCondition
     * @param condition_name Name for the XML tag for this node
     * @param conf BT node configuration
     */
    CheckTopicStateCondition(const std::string& condition_name, const BT::NodeConfiguration& conf);

    CheckTopicStateCondition() = delete;

    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override;

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing node-specific ports
     */
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