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

namespace rmf_robot_client
{
  class NavigationTask : public BaseTask
  {
   public:
    NavigationTask(int task_id, int step, std::shared_ptr<rclcpp::Node> ros_node, double x, double y, double yaw);

    bool start(std::function<void(int)> next_task_callback) override;
    bool cancel();
    std::string get_type();
    bool receive_new_settings(std::string command, std::vector<std::string> value) override;

   private:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    double _target_position_x;
    double _target_position_y;
    double _target_position_yaw;

    std::string _map_frame_id;
    std::string _behavior_tree;

    rclcpp_action::Client<NavigateToPose>::SharedPtr _navigate_to_pose_client;
    GoalHandleNavigateToPose::SharedPtr _current_action_goal_handle;

    void start_navigation();
    void feedback_callback(GoalHandleNavigateToPose::SharedPtr,
                           const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void result_callback(const GoalHandleNavigateToPose::WrappedResult& result);
    void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr& goal_handle);
  };
}   // namespace rmf_robot_client
#endif   //  ROBOT_CLIENT__NAVIGATION_ACTION_HPP_
