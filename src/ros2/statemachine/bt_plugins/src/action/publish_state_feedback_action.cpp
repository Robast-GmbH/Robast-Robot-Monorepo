#include "bt_plugins/action/publish_state_feedback_action.hpp"

namespace statemachine
{
  PublishStateFeedback::PublishStateFeedback(const std::string &name, const BT::NodeConfig &config)
      : BT::SyncActionNode(name, config)
  {
    _node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    std::string topic_name;
    getInput("state_feedback_topic", topic_name);
    _state_feedback_publisher = _node->create_publisher<communication_interfaces::msg::StateFeedback>(topic_name, 10);
  }

  BT::NodeStatus PublishStateFeedback::tick()
  {
    communication_interfaces::msg::StateFeedback state_feedback = communication_interfaces::msg::StateFeedback();
    getInput<std::string>("state_feedback", state_feedback.state);
    getInput("drawer_address", state_feedback.drawer_address);
    _state_feedback_publisher->publish(state_feedback);
    return BT::NodeStatus::SUCCESS;
  }
}   // namespace statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<statemachine::PublishStateFeedback>("PublishStateFeedback");
}