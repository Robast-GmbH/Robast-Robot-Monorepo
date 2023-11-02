#ifndef ROBOT_CLIENT__NAVIGATION_TASK_HPP_
#define ROBOT_CLIENT__NAVIGATION_TASK_HPP_
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>

#include <memory>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

#include "base_task.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_pose.hpp"

namespace rmf_robot_client
{
  class NavigationTask : public BaseTask
  {
   public:
    NavigationTask(TaskId task_id,
                   std::shared_ptr<rclcpp::Node> ros_node,
                   std::shared_ptr<TaskId> task_tracker,
                   RobotPose goal_pose);

    void start() override;
    bool cancel();
    std::string get_type();
    bool receive_new_settings(std::string command, std::vector<std::string> value) override;

   private:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    rclcpp_action::Client<NavigateToPose>::SharedPtr _navigate_to_pose_client;
    GoalHandleNavigateToPose::SharedPtr _current_action_goal_handle;

    RobotPose _target_pose;
    std::string _map_frame_id;
    std::string _behavior_tree;

    void start_navigation();
    void feedback_callback(GoalHandleNavigateToPose::SharedPtr,
                           const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void result_callback(const GoalHandleNavigateToPose::WrappedResult& result);
    void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr& goal_handle);
    void task_done(bool is_completed);
  };
}   // namespace rmf_robot_client
#endif   //  ROBOT_CLIENT__NAVIGATION_ACTION_HPP_
