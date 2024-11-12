#include "bt_plugins/action/robast_error_pub_action.hpp"

namespace statemachine
{
  RobastErrorPub::RobastErrorPub(const std::string &name, const BT::NodeConfig &config)
      : BT::SyncActionNode(name, config)
  {
    _node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    std::string topic_name;
    getInput("topic", topic_name);
    _error_publisher = _node->create_publisher<communication_interfaces::msg::ErrorBaseMsg>(topic_name, 10);
  }

  BT::NodeStatus RobastErrorPub::tick()
  {
    uint16_t error_code;
    getInput<uint16_t>("error_code", error_code);
    std::string error_data;
    getInput<std::string>("error_data", error_data);
    std::string error_description;
    getInput<std::string>("error_description", error_description);

    communication_interfaces::msg::ErrorBaseMsg error_msg;
    error_msg.error_code = error_code;
    error_msg.error_data = error_data;
    error_msg.error_description = error_description;
    _error_publisher->publish(error_msg);
    return BT::NodeStatus::SUCCESS;
  }
}   // namespace statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<statemachine::RobastErrorPub>("RobastErrorPub");
}
