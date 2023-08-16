#ifndef CLICKED_POINT_TO_NAV_POSE__CLICKED_POINT_TO_NAV_POSE_HPP_
#define CLICKED_POINT_TO_NAV_POSE__CLICKED_POINT_TO_NAV_POSE_HPP_

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace robast_utils
{
  class ClickedPointToNavPose : public rclcpp::Node
  {
   public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    ClickedPointToNavPose();

   private:
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr _clicked_point_subscription;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr _nav_to_pose_client;

    void setup_subscriptions();

    void setup_action_client();

    void clicked_point_topic_callback(const geometry_msgs::msg::PointStamped& clicked_point_msg);

    void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr& goal_handle);

    void feedback_callback(GoalHandleNavigateToPose::SharedPtr,
                           const std::shared_ptr<const NavigateToPose::Feedback> feedback);

    void result_callback(const GoalHandleNavigateToPose::WrappedResult& result);
  };

}   // namespace robast_utils

#endif   // CLICKED_POINT_TO_NAV_POSE__CLICKED_POINT_TO_NAV_POSE_HPP_