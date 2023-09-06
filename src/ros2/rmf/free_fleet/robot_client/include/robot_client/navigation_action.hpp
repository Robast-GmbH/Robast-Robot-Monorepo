#ifndef RMF__ROBOT_CLIENT__NAVIGATION_ACTION_HPP_
#define RMF__ROBOT_CLIENT__NAVIGATION_ACTION_HPP_

#include "action.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <math.h>

namespace rmf_robot_client
{
  class NavigationAction : public Action
  {
    private:
      float x;
      float y;
      float yaw;
      std::string behavior_tree;
      std::string frame_id;

      using NavigateToPose = nav2_msgs::action::NavigateToPose;
      using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
      rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;

      void start_navigation();
      void goal_response_callback(std::shared_future<GoalHandleNavigateToPose::SharedPtr> future);
      void feedback_callback(GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);
      void result_callback(const GoalHandleNavigateToPose::WrappedResult &result);

    public:
      NavigationAction(int task_id, int step, std::shared_ptr<rclcpp::Node> ros_node, std::map<std::string, std::string> config, float x, float y, float yaw, std::string behavior_tree, std::string frame_id);
      //~NavigationAction();

      bool start(std::function<void(bool)> next_action_callback);
      bool cancel();
      std::string get_type();
      bool receive_new_settings(std::string command, std::string value);

  };
}// namespace robot_client
#endif   // RMF__ROBOT_CLIENT__NAVIGATION_ACTION_HPP_