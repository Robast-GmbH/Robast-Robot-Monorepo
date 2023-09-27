#ifndef RB_THERON__TASK_SOLUTION_EXECUTOR_HPP_
#define RB_THERON__TASK_SOLUTION_EXECUTOR_HPP_

#include <memory>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <moveit_task_constructor_msgs/msg/solution_info.hpp>
#include <moveit_task_constructor_msgs/msg/sub_solution.hpp>
#include <moveit_task_constructor_msgs/msg/sub_trajectory.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace gazebo_trajectory_executor
{

  class TaskSolutionExecutor
  {
   public:
    using ExecuteTaskSolution = moveit_task_constructor_msgs::action::ExecuteTaskSolution;
    using GoalHandleExecuteTaskSolution = rclcpp_action::ServerGoalHandle<ExecuteTaskSolution>;

    TaskSolutionExecutor(const rclcpp::Node::SharedPtr& node,
                         const std::string& ros_robot_trajectory_topic,
                         const std::string& task_execution_solution_action);
    ~TaskSolutionExecutor(){};

   private:
    rclcpp::Node::SharedPtr _node;

    rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr _ros_robot_trajectory_pub;

    rclcpp_action::Server<ExecuteTaskSolution>::SharedPtr _execute_task_solution_server;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const ExecuteTaskSolution::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecuteTaskSolution> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleExecuteTaskSolution> goal_handle);

    void execute(const std::shared_ptr<GoalHandleExecuteTaskSolution> goal_handle);

    void print_robot_trajectory_msg(const moveit_msgs::msg::RobotTrajectory::SharedPtr& msg);
    void print_joint_trajectory_msg(const trajectory_msgs::msg::JointTrajectory& msg);
    void print_multi_dof_joint_trajectory(const trajectory_msgs::msg::MultiDOFJointTrajectory& msg);
  };
}   // namespace gazebo_trajectory_executor
#endif   // RB_THERON__TASK_SOLUTION_EXECUTOR_HPP_