#include "robot_client/navigation_task.hpp"

namespace rmf_robot_client
{
  NavigationTask::NavigationTask(
      int task_id, int step, std::shared_ptr<rclcpp::Node> ros_node, double x, double y, double yaw)
      : BaseTask(task_id, step, ros_node)
  {
    _x = x;
    _y = y;
    _yaw = yaw;

    _frame_id = ros_node_->get_parameter("map_frame_id").as_string();
    _behavior_tree = ros_node_->get_parameter("behavior_tree").as_string();
    _navigate_to_pose_client = rclcpp_action::create_client<NavigateToPose>(
        ros_node, ros_node_->get_parameter("nav2_navigation_to_pose_action_topic").as_string());
  }

  bool NavigationTask::start(std::function<void(int)> next_task_callback)
  {
    BaseTask::start(next_task_callback);

    RCLCPP_INFO(ros_node_->get_logger(), "start navigation_task");
    finish_task_ = next_task_callback;
    if (!this->_navigate_to_pose_client->wait_for_action_server())
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "Action server not available after waiting");
      // cancel task
      return false;
    }
    start_navigation();
    return true;
  }

  void NavigationTask::start_navigation()
  {
    NavigateToPose::Goal pose_msg = NavigateToPose::Goal();
    pose_msg.pose.header.frame_id = _frame_id;
    pose_msg.pose.header.stamp = ros_node_->now();
    pose_msg.pose.pose.position.x = _x;
    pose_msg.pose.pose.position.y = _y;
    pose_msg.pose.pose.position.z = 0;

    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, _yaw);
    pose_msg.pose.pose.orientation.x = quaternion.getX();
    pose_msg.pose.pose.orientation.y = quaternion.getY();
    pose_msg.pose.pose.orientation.z = quaternion.getZ();
    pose_msg.pose.pose.orientation.w = quaternion.getW();

    pose_msg.behavior_tree = _behavior_tree;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_goal_options =
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        std::bind(&NavigationTask::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&NavigationTask::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&NavigationTask::result_callback, this, std::placeholders::_1);

    this->_navigate_to_pose_client->async_send_goal(pose_msg, send_goal_options);
  }

  bool NavigationTask::cancel()
  {
    this->_navigate_to_pose_client->async_cancel_goal(_current_action_goal_handle);
    publish_task_state("Canceld", "", true);
    finish_task_(step_);
    return true;
  }

  void NavigationTask::goal_response_callback(const GoalHandleNavigateToPose::SharedPtr& goal_handle)
  {
    _current_action_goal_handle = goal_handle;
    if (!_current_action_goal_handle)
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "Goal was rejected by server");
      publish_task_state("Canceld", "could not plan route to goal pose", true);
      finish_task_(step_);
    }
    else
    {
      RCLCPP_INFO(ros_node_->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void NavigationTask::feedback_callback(GoalHandleNavigateToPose::SharedPtr,
                                         const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(ros_node_->get_logger(),
                "navigate_to_pose feedback received. number of recoveries:%i distance:%f ",
                feedback->number_of_recoveries,
                feedback->distance_remaining);
  }

  void NavigationTask::result_callback(const GoalHandleNavigateToPose::WrappedResult&)
  {
    publish_task_state("Completed", "destination_reached", true);
    finish_task_(step_);
  }

  bool NavigationTask::receive_new_settings(std::string command, std::vector<std::string> value)
  {
    if (BaseTask::receive_new_settings(command, value))
    {
      return true;
    }
  }

  std::string NavigationTask::get_type()
  {
    return "NAVIGATION_TASK";
  }

}   // namespace rmf_robot_client
