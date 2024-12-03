#include "bt_plugins/action/evaluate_drive_direction_action.hpp"

namespace statemachine
{
  EvaluateDriveDirection::EvaluateDriveDirection(const std::string &name, const BT::NodeConfig &config)
      : BT::SyncActionNode(name, config)
  {
    getInput("global_path_topic", _global_path_topic_name);
    getInput("cmd_vel_topic", _cmd_vel_topic);
    getInput("prediction_horizon", _prediction_horizon);
    getInput("global_frame", _global_frame);
    getInput("base_frame", _base_frame);

    _node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    _callback_group = _node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    _callback_group_executor.add_callback_group(_callback_group, _node->get_node_base_interface());

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = _callback_group;

    _global_path_sub = _node->create_subscription<nav_msgs::msg::Path>(
      _global_path_topic_name,
      10,
      std::bind(&EvaluateDriveDirection::global_path_callback, this, std::placeholders::_1),
      sub_option);

    _cmd_vel_sub = _node->create_subscription<geometry_msgs::msg::Twist>(
      _cmd_vel_topic,
      10,
      std::bind(&EvaluateDriveDirection::cmd_vel_callback, this, std::placeholders::_1),
      sub_option);

    _timestamp_last_cmd_vel = rclcpp::Time(0);

    _tf = std::make_shared<tf2_ros::Buffer>(_node->get_clock());
    auto timer_interface =
      std::make_shared<tf2_ros::CreateTimerROS>(_node->get_node_base_interface(), _node->get_node_timers_interface());
    _tf->setCreateTimerInterface(timer_interface);
    _transform_listener = std::make_shared<tf2_ros::TransformListener>(*_tf);
  }

  void EvaluateDriveDirection::global_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("EvaluateDriveDirection"), "path received");
    _global_path = *msg;
  }

  void EvaluateDriveDirection::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("EvaluateDriveDirection"), "Cmd vel received. Setting timestamp.");
    _timestamp_last_cmd_vel = _node->now();
  }

  void EvaluateDriveDirection::exposeDriveDirection()
  {
    RCLCPP_DEBUG(rclcpp::get_logger("EvaluateDriveDirection"), "path size: %d", _global_path.poses.size());

    const builtin_interfaces::msg::Time current_time = _node->now();

    if (is_robot_sleeping(current_time))
    {
      return;
    }

    if (is_robot_standing(current_time))
    {
      return;
    }

    if (!nav2_util::getCurrentPose(_global_pose, *_tf, _global_frame, _base_frame, 0.2) ||
        !(_global_path.poses.size() > 0))
    {
      RCLCPP_WARN(rclcpp::get_logger("EvaluateDriveDirection"), "Could not get current pose");
      return;
    }

    _current_path_index = getCurrentIndex(_global_pose.pose, _global_path);
    _global_path.poses.erase(_global_path.poses.begin(), _global_path.poses.begin() + _current_path_index);
    if (_global_path.poses.size() > _prediction_horizon)
    {
      auto start_pose = _global_path.poses[0].pose;
      auto end_pose = _global_path.poses[_prediction_horizon].pose;

      _direction = utils::DirectionToString(utils::calculateDirection(start_pose, end_pose));
    }
    // TODO @TAlscher add something to handle the case where the path is too short
    else
    {
      _direction = "standing";
    }
    RCLCPP_DEBUG(rclcpp::get_logger("EvaluateDriveDirection"), "direction: %s", _direction.c_str());
    setOutput("direction", _direction);
  }

  bool EvaluateDriveDirection::is_robot_standing(const builtin_interfaces::msg::Time current_time)
  {
    if (current_time.sec - _timestamp_last_cmd_vel.sec > STANDING_THRESHOLD_IN_S)
    {
      RCLCPP_DEBUG(rclcpp::get_logger("EvaluateDriveDirection"),
                   "Cmd vel is outdated for over %d seconds",
                   STANDING_THRESHOLD_IN_S);
      setOutput("direction", "standing");
      return true;
    }
    return false;
  }

  bool EvaluateDriveDirection::is_robot_sleeping(const builtin_interfaces::msg::Time current_time)
  {
    if (_timestamp_last_cmd_vel.sec == 0)
    {
      RCLCPP_DEBUG(rclcpp::get_logger("EvaluateDriveDirection"), "Cmd vel has not been received yet");
      setOutput("direction", "sleeping");
      return true;
    }

    if (current_time.sec - _timestamp_last_cmd_vel.sec > SLEEPING_THRESHOLD_IN_S)
    {
      RCLCPP_DEBUG(rclcpp::get_logger("EvaluateDriveDirection"),
                   "Cmd vel is outdated for over %d seconds so entering state sleep",
                   SLEEPING_THRESHOLD_IN_S);
      setOutput("direction", "sleeping");
      return true;
    }
    return false;
  }

  int EvaluateDriveDirection::getCurrentIndex(const geometry_msgs::msg::Pose &current_pose,
                                              const nav_msgs::msg::Path &path)
  {
    int closest_index = -1;
    double min_distance = std::numeric_limits<double>::max();
    RCLCPP_DEBUG(rclcpp::get_logger("EvaluateDriveDirection"),
                 "current pose: %f %f",
                 current_pose.position.x,
                 current_pose.position.y);

    for (size_t i = 0; i < path.poses.size(); ++i)
    {
      double distance = std::hypot(current_pose.position.x - path.poses[i].pose.position.x,
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

}   // namespace statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<statemachine::EvaluateDriveDirection>("EvaluateDriveDirection");
}