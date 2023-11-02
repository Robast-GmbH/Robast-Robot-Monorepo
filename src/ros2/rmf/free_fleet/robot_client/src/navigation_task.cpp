#include "robot_client/navigation_task.hpp"

namespace rmf_robot_client
{
  NavigationTask::NavigationTask(TaskId task_id,
                                 std::shared_ptr<rclcpp::Node> ros_node,
                                 std::shared_ptr<TaskId> task_indicator,
                                 RobotPose goal_pose)
      : BaseTask(task_id, ros_node, task_indicator)
  {
    _target_pose = goal_pose;
    _map_frame_id = ros_node->get_parameter("map_frame_id").as_string();
    _behavior_tree = ros_node->get_parameter("nav_behavior_tree").as_string();
    _navigate_to_pose_client = rclcpp_action::create_client<NavigateToPose>(
        ros_node, ros_node->get_parameter("nav2_navigation_to_pose_action_topic").as_string());
  }

  void NavigationTask::start()
  {
    RCLCPP_INFO(ros_node_->get_logger(), "start navigation_task");
    if (!this->_navigate_to_pose_client->wait_for_action_server())
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "Action server not available after waiting");
      return;
    }
    start_navigation();
    return;
  }

  void NavigationTask::start_navigation()
  {
    NavigateToPose::Goal pose_msg = NavigateToPose::Goal();
    pose_msg.pose.header.frame_id = _map_frame_id;
    pose_msg.pose.header.stamp = ros_node_->now();
    pose_msg.pose.pose.position.x = _target_pose.x_pose;
    pose_msg.pose.pose.position.y = _target_pose.y_pose;
    pose_msg.pose.pose.position.z = 0;

    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, _target_pose.yaw_pose);
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

    RCLCPP_INFO(ros_node_->get_logger(), "navigation started");
    this->_navigate_to_pose_client->async_send_goal(pose_msg, send_goal_options);
  }

  void NavigationTask::goal_response_callback(const GoalHandleNavigateToPose::SharedPtr& goal_handle)
  {
    _current_action_goal_handle = goal_handle;
    if (!_current_action_goal_handle)
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "Goal was rejected by server");
      publish_task_state("Canceld", "could not plan route to goal pose", true);
      task_done(false);
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
                "navigate_to_pose feedback received. number of recoveries:%i, distance:%i, time to goal:%i",
                feedback->number_of_recoveries,
                feedback->number_of_recoveries,
                feedback->estimated_time_remaining.sec);
    publish_task_state("EstimatedTimeRemaining", feedback->estimated_time_remaining.sec + "", false);
  }

  void NavigationTask::result_callback(const GoalHandleNavigateToPose::WrappedResult&)
  {
    publish_task_state("Completed", "destination_reached", true);
    task_done(true);
    return;
  }

  bool NavigationTask::receive_new_settings(std::string command, std::vector<std::string> value)
  {
    return BaseTask::receive_new_settings(command, value);
  }

  bool NavigationTask::cancel()
  {
    this->_navigate_to_pose_client->async_cancel_goal(_current_action_goal_handle);
    publish_task_state("Canceld", "", true);
    task_done(false);
    return true;
  }

  void NavigationTask::task_done(bool is_completed)
  {
    _navigate_to_pose_client.reset();
    if (is_completed)
    {
      start_next_phase();
    }
  }


  std::string NavigationTask::get_type()
  {
    return "NAVIGATION_TASK";
  }

}   // namespace rmf_robot_client
