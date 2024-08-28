#include "bt_plugins/action/evaluate_drive_direction_action.hpp"

namespace statemachine
{
  EvaluateDriveDirection::EvaluateDriveDirection(
      const std::string &name,
      const BT::NodeConfig &config)
      : BT::SyncActionNode(name, config)
  {
    getInput("path_topic", _topic_name);
    getInput("prediction_horizon", _prediction_horizon);
    getInput("global_frame", _global_frame);
    getInput("base_frame", _base_frame);
    _node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    _callback_group = _node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    _callback_group_executor.add_callback_group(_callback_group, _node->get_node_base_interface());

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = _callback_group;
    _drawer_open_sub = _node->create_subscription<nav_msgs::msg::Path>(
        _topic_name,
        10,
        std::bind(&EvaluateDriveDirection::callbackPathReceived, this, std::placeholders::_1),
        sub_option);
    _tf = std::make_shared<tf2_ros::Buffer>(_node->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        _node->get_node_base_interface(),
        _node->get_node_timers_interface());
    _tf->setCreateTimerInterface(timer_interface);
    _transform_listener = std::make_shared<tf2_ros::TransformListener>(*_tf);
  }

  void EvaluateDriveDirection::callbackPathReceived(const nav_msgs::msg::Path::SharedPtr msg)
  {
    RCLCPP_INFO(rclcpp::get_logger("EvaluateDriveDirection"), "path received");
    _path = *msg;
  }

  void EvaluateDriveDirection::exposeDriveDirection()
  {

    RCLCPP_DEBUG(rclcpp::get_logger("EvaluateDriveDirection"), "path size: %d", _path.poses.size());
    if (!nav2_util::getCurrentPose(_global_pose, *_tf, _global_frame, _base_frame, 0.2) ||
        !(_path.poses.size() > 0))
    {
      RCLCPP_WARN(rclcpp::get_logger("EvaluateDriveDirection"), "Could not get current pose");
      return;
    }

    _current_path_index = getCurrentIndex(_global_pose.pose, _path);
    _path.poses.erase(_path.poses.begin(), _path.poses.begin() + _current_path_index);
    if (_path.poses.size() > _prediction_horizon)
    {
      auto start_pose = _path.poses[0].pose;
      auto end_pose = _path.poses[_prediction_horizon].pose;

      _direction = utils::DirectionToString(utils::calculateDirection(start_pose, end_pose));
    }
    else if (_path.poses.size() > CLOSE_TO_TARGET_SHOULD_BE_CHANGED_REGULARLY_AND_DELETED_IN_THE_END_BEFOR_FINAL_DEPLOY_BUT_IS_CURRENTRLY_A_KINDA_OK_NUMBER)
    {      
      auto start_pose = _path.poses[0].pose;
      auto end_pose = _path.poses[_path.poses.size() - 1].pose;

      _direction = utils::DirectionToString(utils::calculateDirection(start_pose, end_pose));
    }
    else
    {
      _direction = "standing";
    }
    RCLCPP_DEBUG(rclcpp::get_logger("EvaluateDriveDirection"), "direction: %s", _direction.c_str());
    setOutput("direction", _direction);
  }

  int EvaluateDriveDirection::getCurrentIndex(const geometry_msgs::msg::Pose &current_pose, const nav_msgs::msg::Path &path)
  {
    int closest_index = -1;
    double min_distance = std::numeric_limits<double>::max();
    RCLCPP_DEBUG(rclcpp::get_logger("EvaluateDriveDirection"), "current pose: %f %f", current_pose.position.x, current_pose.position.y);

    for (size_t i = 0; i < path.poses.size(); ++i)
    {
      double distance = std::hypot(
          current_pose.position.x - path.poses[i].pose.position.x,
          current_pose.position.y - path.poses[i].pose.position.y);

      if (distance < min_distance)
      {
        min_distance = distance;
        closest_index = i;
      }
    }

    return closest_index;
  }

  BT::NodeStatus EvaluateDriveDirection::tick()
  {
    RCLCPP_DEBUG(rclcpp::get_logger("EvaluateDriveDirection"), "EvaluateDriveDirection tick");
    _callback_group_executor.spin_some();
    exposeDriveDirection();
    return BT::NodeStatus::SUCCESS;
  }

}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<statemachine::EvaluateDriveDirection>("EvaluateDriveDirection");
}