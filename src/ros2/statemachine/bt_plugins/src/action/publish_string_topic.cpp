#include "bt_plugins/action/publish_string_topic.hpp"

namespace statemachine
{
  PublishStringTopic::PublishStringTopic(const std::string &name, const BT::NodeConfig &config)
      : BT::SyncActionNode(name, config)
  {
    _node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    std::string topic_name;
    getInput("topic", topic_name);
    _publisher = _node->create_publisher<std_msgs::msg::String>(topic_name, 10);
  }

  BT::NodeStatus PublishStringTopic::tick()
  {
    std_msgs::msg::String msg;
    getInput<std::string>("message", msg.data);
    _publisher->publish(msg);
    return BT::NodeStatus::SUCCESS;
  }
}   // namespace statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<statemachine::PublishStringTopic>("PublishStringTopic");
}