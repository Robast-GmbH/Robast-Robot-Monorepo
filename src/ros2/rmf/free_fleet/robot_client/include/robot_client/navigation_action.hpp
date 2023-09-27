#ifndef RMF__ROBOT_CLIENT__NAVIGATION_ACTION_HPP_
#define RMF__ROBOT_CLIENT__NAVIGATION_ACTION_HPP_

#include "action.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
 #include <tf2/LinearMath/Quaternion.h>
#include <math.h>

namespace rmf_robot_client
{
  class NavigationAction : public Action
  {
    private: 
      using NavigateToPose = nav2_msgs::action::NavigateToPose;
      using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

      float x;
      float y;
      float yaw;

      std::string frame_id;
      std::string behavior_tree;

      rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
      GoalHandleNavigateToPose::SharedPtr current_action_goal_handle;

      void start_navigation();
      void feedback_callback(GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);
      void result_callback(const GoalHandleNavigateToPose::WrappedResult &result);
      void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr &goal_handle);

    public:
      NavigationAction(int task_id, int step, std::shared_ptr<rclcpp::Node> ros_node, float x, float y, float yaw);
      //~NavigationAction();

      bool start(std::function<void(int)> next_action_callback);
      bool cancel();
      std::string get_type();
      bool receive_new_settings(std::string command, std::vector<std::string> value) override;

  };
}// namespace robot_client
#endif   // RMF__ROBOT_CLIENT__NAVIGATION_ACTION_HPP_