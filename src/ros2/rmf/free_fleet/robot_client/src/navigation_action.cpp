#include "robot_client/navigation_action.hpp"

namespace rmf_robot_client
{
  NavigationAction::NavigationAction(int task_id, int step, std::shared_ptr<rclcpp::Node>  ros_node, std::map<std::string,std::string> config,  float x, float y, float yaw, std::string behavior_tree, std::string frame_id):Action(task_id, step, ros_node, config)
  {
    this->x = x;
    this->y = y;
    this->yaw = yaw;
    this->behavior_tree = behavior_tree;
    navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(ros_node, config["nav2_navigation_to_pose_action_topic"]);
  }
  
  bool NavigationAction::start(std::function<void(bool)> next_action_callback)
  {
    RCLCPP_INFO(ros_node->get_logger(), "start drawer_action");
    finish_action = next_action_callback;
     if (!this->navigate_to_pose_client_->wait_for_action_server()) {
      RCLCPP_ERROR(ros_node->get_logger(), "Action server not available after waiting");
      //cancel task
      return false;
     }
     start_navigation();
     return true;
  }

  void NavigationAction::start_navigation()
  {
    NavigateToPose::Goal pose_msg = NavigateToPose::Goal();
    pose_msg.pose.header.frame_id= frame_id;
    pose_msg.pose.header.stamp = ros_node->now();
    pose_msg.pose.pose.position.x = x;
    pose_msg.pose.pose.position.y = y;
    pose_msg.pose.pose.position.z = 0;

    double yaw_radiant = yaw * M_PI / 180;
    pose_msg.pose.pose.orientation.x = cos(yaw_radiant / 2);
    pose_msg.pose.pose.orientation.y = 0;
    pose_msg.pose.pose.orientation.z = 0;
    pose_msg.pose.pose.orientation.w = sin(yaw_radiant/2);

    pose_msg.behavior_tree = behavior_tree;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    
    //send_goal_options.goal_response_callback = std::bind(&NavigationAction::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&NavigationAction::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&NavigationAction::result_callback, this, std::placeholders::_1);
 
    this->navigate_to_pose_client_->async_send_goal(pose_msg, send_goal_options);
  }

  bool NavigationAction::cancel()
  {
    this->navigate_to_pose_client_->async_cancel_goal(current_action_goal_handle);
    publish_task_state("Canceld", "", true);
    finish_action(false);
    return true;
  }
  

  void NavigationAction::goal_response_callback(std::shared_future<GoalHandleNavigateToPose::SharedPtr> future)
  {
    current_action_goal_handle= future.get();
    if (!current_action_goal_handle)
    {
      RCLCPP_ERROR(ros_node->get_logger(), "Goal was rejected by server");
      publish_task_state("Canceld", "could not plan route to goal pose", true);
      finish_action(false);
    }
    else
    {
      RCLCPP_INFO(ros_node->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void NavigationAction::feedback_callback(GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(ros_node->get_logger(), "navigate_to_pose feedback recived");
  }

  void NavigationAction::result_callback(const GoalHandleNavigateToPose::WrappedResult &result)
  {
    publish_task_state("Completed", "destination_reached", true);
    finish_action(true);
  }

  bool NavigationAction::receive_new_settings(std::string command, std::string value)
  {

  }

  std::string NavigationAction::get_type()
  {
    return "NAVIGATION_ACTION";
  }

}