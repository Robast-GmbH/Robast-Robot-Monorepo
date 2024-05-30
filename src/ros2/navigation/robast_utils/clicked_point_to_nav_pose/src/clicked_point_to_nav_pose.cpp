#include "clicked_point_to_nav_pose/clicked_point_to_nav_pose.hpp"

namespace robast_utils
{
  ClickedPointToNavPose::ClickedPointToNavPose() : Node("clicked_point_to_nav_pose")
  {
    setup_subscriptions();
    setup_action_client();
  }

  void ClickedPointToNavPose::setup_subscriptions()
  {
    _clicked_point_subscription = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "clicked_point", 1, std::bind(&ClickedPointToNavPose::clicked_point_topic_callback, this, std::placeholders::_1));
  }

  void ClickedPointToNavPose::setup_action_client()
  {
    _nav_to_pose_client = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  }

  void ClickedPointToNavPose::clicked_point_topic_callback(const geometry_msgs::msg::PointStamped& clicked_point_msg)
  {
    RCLCPP_INFO(this->get_logger(),
                "Received clicked_point with coordinates (%f, %f, %f)",
                clicked_point_msg.point.x,
                clicked_point_msg.point.y,
                clicked_point_msg.point.z);

    if (!this->_nav_to_pose_client->wait_for_action_server())
    {
      RCLCPP_ERROR(this->get_logger(), "Navigate to Pose action server not available after waiting");
      return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header = clicked_point_msg.header;
    goal_msg.pose.pose.position = clicked_point_msg.point;
    goal_msg.behavior_tree =
      "/workspace/src/navigation/nav_bringup/behavior_trees/humble/"
      "navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml";

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&ClickedPointToNavPose::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&ClickedPointToNavPose::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&ClickedPointToNavPose::result_callback, this, std::placeholders::_1);
    _nav_to_pose_client->async_send_goal(goal_msg, send_goal_options);
  }

  void ClickedPointToNavPose::goal_response_callback(const GoalHandleNavigateToPose::SharedPtr& goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void ClickedPointToNavPose::feedback_callback(GoalHandleNavigateToPose::SharedPtr,
                                                const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
  }

  void ClickedPointToNavPose::result_callback(const GoalHandleNavigateToPose::WrappedResult& result)
  {
  }

}   // namespace robast_utils
