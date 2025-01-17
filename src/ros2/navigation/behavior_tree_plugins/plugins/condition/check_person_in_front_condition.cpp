#include "behavior_tree_plugins/plugins/condition/check_person_in_front_condition.hpp"

namespace nav2_behavior_tree
{

  CheckPersonInFrontCondition::CheckPersonInFrontCondition(const std::string& name, const BT::NodeConfiguration& conf)
      : BT::ConditionNode(name, conf)
  {
    _detection_data.point.x = -1.0;
    _node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    _callback_group = _node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    _callback_group_executor.add_callback_group(_callback_group, _node->get_node_base_interface());
    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = _callback_group;

    std::string topic_name = "/person_detection";
    getInput("topic", topic_name);
    _detection_subscriber = _node->create_subscription<geometry_msgs::msg::PointStamped>(
      topic_name,
      10,
      std::bind(&CheckPersonInFrontCondition::detection_callback, this, std::placeholders::_1),
      sub_option);
  }

 

  BT::NodeStatus CheckPersonInFrontCondition::tick()
  {
    _callback_group_executor.spin_some();
    float detection_range = 0.0;
    if (!getInput("detection_range", detection_range))
    {
      throw BT::RuntimeError("missing required input [detection_range]");
    }
    // Assuming the sensor data is from a range sensor
    if (_detection_data.point.x <= 0.0 || _detection_data.header.frame_id == "")
    {
      RCLCPP_DEBUG(_node->get_logger(), "No person detected in front");
      return BT::NodeStatus::FAILURE;
    }

    if (_detection_data.point.x < detection_range)
    {
      RCLCPP_DEBUG(_node->get_logger(), "Person detected in front at distance: %f", _detection_data.point.x);
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      RCLCPP_DEBUG(_node->get_logger(), "Person detected in front at distance: %f", _detection_data.point.x);
      return BT::NodeStatus::FAILURE;
    }
  }

  void CheckPersonInFrontCondition::detection_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    _detection_data = *msg;
  }

}   // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::CheckPersonInFrontCondition>("CheckPersonInFront");
}