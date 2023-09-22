#include "gazebo_controller_manager/task_solution_executor.hpp"

namespace gazebo_controller_manager
{
  TaskSolutionExecutor::TaskSolutionExecutor(const rclcpp::Node::SharedPtr& node,
                                             const std::string& ros_robot_trajectory_topic,
                                             const std::string& task_execution_solution_action)
  {
    RCLCPP_INFO(node->get_logger(), "Starting TaskSolutionExecutor!");

    _node = node;

    _execute_task_solution_server =
        rclcpp_action::create_server<ExecuteTaskSolution>(this,
                                                          task_execution_solution_action,
                                                          std::bind(&TaskSolutionExecutor::handle_goal, this, _1, _2),
                                                          std::bind(&TaskSolutionExecutor::handle_cancel, this, _1),
                                                          std::bind(&TaskSolutionExecutor::handle_accepted, this, _1));

    _ros_robot_trajectory_pub =
        _node->create_publisher<moveit_msgs::msg::RobotTrajectory>(ros_robot_trajectory_topic, 10);
  }

  rclcpp_action::GoalResponse TaskSolutionExecutor::handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                                std::shared_ptr<const ExecuteTaskSolution::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    (void) uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse TaskSolutionExecutor::handle_cancel(
      const std::shared_ptr<GoalHandleExecuteTaskSolution> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void) goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void TaskSolutionExecutor::handle_accepted(const std::shared_ptr<GoalHandleExecuteTaskSolution> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&TaskSolutionExecutor::execute, this, _1), goal_handle}.detach();
  }

  void TaskSolutionExecutor::execute(const std::shared_ptr<GoalHandleExecuteTaskSolution> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ExecuteTaskSolution::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i)
    {
      // Check if there is a cancel request
      if (goal_handle->is_canceling())
      {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok())
    {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

}   // namespace gazebo_controller_manager