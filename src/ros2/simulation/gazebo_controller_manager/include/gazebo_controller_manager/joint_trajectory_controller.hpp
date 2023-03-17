#ifndef RB_THERON__JOINT_TRAJECTORY_CONTROLLER_HPP_
#define RB_THERON__JOINT_TRAJECTORY_CONTROLLER_HPP_

#include <condition_variable>
#include <gz/msgs.hh>
#include <gz/transport/Node.hh>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unordered_map>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace gazebo_controller_manager
{
  class JointTrajectoryController : public rclcpp::Node
  {
   public:
    JointTrajectoryController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~JointTrajectoryController(){};

   private:
    void initialize_gz_transport_node(std::vector<std::string> joint_names, std::vector<std::string> gz_joint_topics);

    void set_joint_trajectory_cb(const trajectory_msgs::msg::JointTrajectory::SharedPtr);

    void update_joint_position_timer_cb();

    void update_cmd_vel_for_mobile_base(double c);

    void handle_finished_trajectory_execution();

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>>
            goal_handle);

    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>>
            goal_handle);

    void execute_follow_joint_trajectory(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>>
            goal_handle);

    rclcpp_action::GoalResponse handle_execute_trajectory_goal(
        const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const moveit_msgs::action::ExecuteTrajectory::Goal> goal);

    rclcpp_action::CancelResponse handle_execute_trajectory_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::ExecuteTrajectory>> goal_handle);

    void handle_execute_trajectory_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::ExecuteTrajectory>> goal_handle);

    void execute_trajectory(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::ExecuteTrajectory>> goal_handle);

    bool is_trajectory_motion_finished();

    std::vector<std::string> get_gz_cmd_joint_topics(std::vector<std::string> joint_names);

    bool is_joint_order_correct(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

    void reverse_joint_order();

    void set_mobile_base_trajectory(trajectory_msgs::msg::MultiDOFJointTrajectory mobile_base_trajectory);

    bool received_action_from_execute_trajectory();

   private:
    std::shared_ptr<gz::transport::Node> gz_transport_node_;
    rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr
        follow_joint_trajectory_action_server_;
    rclcpp_action::Server<moveit_msgs::action::ExecuteTrajectory>::SharedPtr execute_trajectory_action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    // gz pub
    std::vector<std::shared_ptr<gz::transport::Node::Publisher>> gz_cmd_joint_pubs_;
    rclcpp::TimerBase::SharedPtr update_position_timer_;
    // joint names and map
    std::vector<std::string> joint_names_;
    size_t joint_num_;
    std::vector<double> target_joint_positions_;
    std::unordered_map<std::string, int> joint_names_map_;
    std::mutex trajectory_mutex_;
    std::mutex cv_mutex_;
    std::condition_variable cv_;
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> joint_trajectory_points_;
    std::vector<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint> mobile_base_trajectory_points_;

    rclcpp::Time trajectory_start_time_;
    unsigned int trajectory_index_;
    bool has_trajectory_{false};
    bool has_trajectory_for_mobile_base_{false};
    bool received_execute_trajectory_{false};
  };
}   // namespace gazebo_controller_manager
#endif   // RB_THERON__JOINT_TRAJECTORY_CONTROLLER_HPP_