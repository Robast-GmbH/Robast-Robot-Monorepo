#include "robot_client/navigation_action.hpp"

namespace rmf_robot_client
{
  NavigationAction::NavigationAction(int task_id, int step, std::shared_ptr<rclcpp::Node>  ros_node, float x, float y, float yaw):Action(task_id, step, ros_node)
  {
    x_ = x;
    y_ = y;
    yaw_ = yaw;

    frame_id_= ros_node_->get_parameter("map_frame_id").as_string();
    behavior_tree_ = ros_node_->get_parameter("behavior_tree").as_string();
    navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(ros_node, ros_node_->get_parameter("nav2_navigation_to_pose_action_topic").as_string());
  }
  
  bool NavigationAction::start(std::function<void(int)> next_action_callback) 
  {
    Action::start(next_action_callback); 

    RCLCPP_INFO(ros_node_->get_logger(), "start navigation_action");
    finish_action = next_action_callback;
    if (!this->navigate_to_pose_client_->wait_for_action_server()) 
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "Action server not available after waiting");
      //cancel task
      return false;
    }
    start_navigation();
    return true;
  }

  void NavigationAction::start_navigation()
  {
    
    NavigateToPose::Goal pose_msg = NavigateToPose::Goal();
    pose_msg.pose.header.frame_id = frame_id_;
    pose_msg.pose.header.stamp = ros_node_->now();
    pose_msg.pose.pose.position.x = x_;
    pose_msg.pose.pose.position.y = y_;
    pose_msg.pose.pose.position.z = 0;

    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, yaw_);
    pose_msg.pose.pose.orientation.x = quaternion.x();
    pose_msg.pose.pose.orientation.y = quaternion.y();
    pose_msg.pose.pose.orientation.z = quaternion.z();
    pose_msg.pose.pose.orientation.w = quaternion.w();

    pose_msg.behavior_tree = behavior_tree_;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    
    send_goal_options.goal_response_callback = std::bind(&NavigationAction::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&NavigationAction::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&NavigationAction::result_callback, this, std::placeholders::_1);
 
    this->navigate_to_pose_client_->async_send_goal(pose_msg, send_goal_options);
  }

  bool NavigationAction::cancel()
  {
    this->navigate_to_pose_client_->async_cancel_goal(current_action_goal_handle_);
    publish_task_state("Canceld", "", true);
    finish_action(step_);
    return true;
  }
  

  void NavigationAction::goal_response_callback(const GoalHandleNavigateToPose::SharedPtr& goal_handle)
  {
    current_action_goal_handle_= goal_handle;
    if (!current_action_goal_handle_)
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "Goal was rejected by server");
      publish_task_state("Canceld", "could not plan route to goal pose", true);
      finish_action(step_);
    }
    else
    {
      RCLCPP_INFO(ros_node_->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void NavigationAction::feedback_callback(GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(ros_node_->get_logger(), "navigate_to_pose feedback received. number of recoveries:%i distance:%f ",feedback->number_of_recoveries, feedback->distance_remaining);
  }

  void NavigationAction::result_callback(const GoalHandleNavigateToPose::WrappedResult &result)
  {
    publish_task_state("Completed", "destination_reached", true);
    finish_action(step_);
  }

  bool NavigationAction::receive_new_settings(std::string command, std::vector<std::string> value)
  {
    if(Action::receive_new_settings(command,value))
    {
      return true;
    }
  }

  std::string NavigationAction::get_type()
  {
    return "NAVIGATION_ACTION";
  }

}