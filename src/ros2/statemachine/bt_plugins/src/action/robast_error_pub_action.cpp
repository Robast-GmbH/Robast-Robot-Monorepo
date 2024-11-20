#include "bt_plugins/action/robast_error_pub_action.hpp"

namespace statemachine
{
  RobastErrorPub::RobastErrorPub(const std::string &name, const BT::NodeConfig &config)
      : BT::SyncActionNode(name, config)
  {
    _node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    std::string topic_name;
    getInput("topic", topic_name);

    rclcpp::QoS qos_error_msgs = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
    qos_error_msgs.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_error_msgs.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos_error_msgs.avoid_ros_namespace_conventions(false);

    _error_publisher = _node->create_publisher<communication_interfaces::msg::ErrorBaseMsg>(topic_name, qos_error_msgs);
  }

  BT::NodeStatus RobastErrorPub::tick()
  {
    communication_interfaces::msg::ErrorBaseMsg error_msg;
    getInput<uint16_t>("error_code", error_msg.error_code);
    getInput<std::string>("error_data", error_msg.error_data);
    getInput<std::string>("error_description", error_msg.error_description);
    _error_publisher->publish(error_msg);
    return BT::NodeStatus::SUCCESS;
  }
}   // namespace statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<statemachine::RobastErrorPub>("RobastErrorPub");
}
