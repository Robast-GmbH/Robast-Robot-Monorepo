#include "bt_plugins/action/evaluate_drive_direction_action.hpp"

namespace statemachine
{
  EvaluateDriveDirection::EvaluateDriveDirection(
      const std::string &name,
      const BT::NodeConfig &config)
      : BT::StatefulActionNode(name, config)
  {
    _node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    getInput("path_topic", _topic_name);
    _callback_group = _node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    _callback_group_executor.add_callback_group(_callback_group, _node->get_node_base_interface());

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = _callback_group;
    _drawer_open_sub = _node->create_subscription<nav_msgs::msg::Path>(
        _topic_name,
        10,
        std::bind(&EvaluateDriveDirection::callbackPathReceived, this, std::placeholders::_1),
        sub_option);
    _timer = _node->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&EvaluateDriveDirection::exposeDriveDirectionTimerCallback, this));
    _path = nav_msgs::msg::Path();
  }

  void EvaluateDriveDirection::callbackPathReceived(const nav_msgs::msg::Path::SharedPtr msg)
  {
    _path = *msg;
  }

  void EvaluateDriveDirection::exposeDriveDirectionTimerCallback()
  {
    if (_path.poses.size() > 60)
    {
      auto start_pose = _path.poses[0].pose.position;
      auto end_pose = _path.poses[60].pose.position;

      _direction = utils::DirectionToString(utils::calculateDirection(_path.poses[0].pose, _path.poses[60].pose));
    }
    else
    {
      _direction = "standing";
    }
    _path = nav_msgs::msg::Path();
    setOutput("direction", _direction);
  }
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<statemachine::EvaluateDriveDirection>("EvaluateDriveDirection");
}