#include "gazebo_controller_manager/task_solution_executor.hpp"

namespace gazebo_controller_manager
{
  TaskSolutionExecutor::TaskSolutionExecutor(const rclcpp::Node::SharedPtr& node,
                                             const std::string& ros_robot_trajectory_topic,
                                             const std::string& task_execution_solution_action)
  {
    RCLCPP_INFO(node->get_logger(), "Starting TaskSolutionExecutor!");

    _node = node;

    _execute_task_solution_server = rclcpp_action::create_server<ExecuteTaskSolution>(
        _node->get_node_base_interface(),
        _node->get_node_clock_interface(),
        _node->get_node_logging_interface(),
        _node->get_node_waitables_interface(),
        task_execution_solution_action,
        std::bind(&TaskSolutionExecutor::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&TaskSolutionExecutor::handle_cancel, this, std::placeholders::_1),
        std::bind(&TaskSolutionExecutor::handle_accepted, this, std::placeholders::_1));

    _ros_robot_trajectory_pub =
        _node->create_publisher<moveit_msgs::msg::RobotTrajectory>(ros_robot_trajectory_topic, 10);
  }

  rclcpp_action::GoalResponse TaskSolutionExecutor::handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                                std::shared_ptr<const ExecuteTaskSolution::Goal> goal)
  {
    RCLCPP_INFO(_node->get_logger(), "Received goal request for executing task solution!");
    (void) uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse TaskSolutionExecutor::handle_cancel(
      const std::shared_ptr<GoalHandleExecuteTaskSolution> goal_handle)
  {
    RCLCPP_INFO(_node->get_logger(), "Received request to cancel goal");
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
    RCLCPP_INFO(_node->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();

    moveit_task_constructor_msgs::msg::Solution mtc_solution = goal->solution;

    uint8_t id_of_solution_with_lowest_cost = get_id_of_solution_with_lowest_cost(
        std::make_shared<moveit_task_constructor_msgs::msg::Solution>(mtc_solution));

    // for (const auto& sub_trajectory : mtc_solution.sub_trajectory)
    // {
    //   print_robot_trajectory_msg(std::make_shared<moveit_msgs::msg::RobotTrajectory>(sub_trajectory.trajectory));
    // }

    moveit_task_constructor_msgs::msg::SubTrajectory sub_trajectory =
        mtc_solution.sub_trajectory[id_of_solution_with_lowest_cost];

    _ros_robot_trajectory_pub->publish(sub_trajectory.trajectory);

    auto result = std::make_shared<ExecuteTaskSolution::Result>();

    // Check if there is a cancel request
    if (goal_handle->is_canceling())
    {
      goal_handle->canceled(result);
      RCLCPP_INFO(_node->get_logger(), "Goal canceled");
      return;
    }

    loop_rate.sleep();

    // Check if goal is done
    if (rclcpp::ok())
    {
      goal_handle->succeed(result);
      RCLCPP_INFO(_node->get_logger(), "Goal succeeded");
    }
  }

  uint8_t TaskSolutionExecutor::get_id_of_solution_with_lowest_cost(
      moveit_task_constructor_msgs::msg::Solution::SharedPtr mtc_solution)
  {
    float lowest_cost = std::numeric_limits<float>::max();
    uint8_t id_of_solution_with_lowest_cost = 0;
    uint8_t id = 0;

    for (const auto& sub_solution : mtc_solution->sub_solution)
    {
      ++id;
      moveit_task_constructor_msgs::msg::SolutionInfo solution_info = sub_solution.info;
      if (solution_info.cost < lowest_cost)
      {
        lowest_cost = solution_info.cost;
        id_of_solution_with_lowest_cost = id;
      }
    }
    RCLCPP_INFO(_node->get_logger(),
                "The solution with the lowest costs (%f) is the one with id %d",
                lowest_cost,
                id_of_solution_with_lowest_cost);
    return id_of_solution_with_lowest_cost;
  }

  void TaskSolutionExecutor::print_robot_trajectory_msg(const moveit_msgs::msg::RobotTrajectory::SharedPtr& msg)
  {
    if (!msg)
    {
      RCLCPP_WARN(_node->get_logger(), "Empty joint trajectory message received");
      return;
    }

    RCLCPP_INFO(_node->get_logger(), "-----------------");

    print_joint_trajectory_msg(msg->joint_trajectory);

    print_multi_dof_joint_trajectory(msg->multi_dof_joint_trajectory);
  }

  void TaskSolutionExecutor::print_joint_trajectory_msg(const trajectory_msgs::msg::JointTrajectory& msg)
  {
    RCLCPP_INFO(_node->get_logger(), "Joint Trajectory:");

    // Print the header information
    RCLCPP_INFO(_node->get_logger(), "Header:");
    RCLCPP_INFO(_node->get_logger(), "  stamp: %f", msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec);
    RCLCPP_INFO(_node->get_logger(), "  frame_id: %s", msg.header.frame_id.c_str());

    // Print the joint names
    RCLCPP_INFO(_node->get_logger(), "Joint Names:");
    for (const auto& name : msg.joint_names)
    {
      RCLCPP_INFO(_node->get_logger(), "  %s", name.c_str());
    }

    // Print the points in the trajectory
    RCLCPP_INFO(_node->get_logger(), "Trajectory Points (number of points: %zu):", msg.points.size());
    for (size_t i = 0; i < msg.points.size(); ++i)
    {
      const auto& point = msg.points[i];

      RCLCPP_INFO(_node->get_logger(), "  Point %zu:", i);
      RCLCPP_INFO(_node->get_logger(),
                  "    time_from_start: %f",
                  point.time_from_start.sec + 1e-9 * point.time_from_start.nanosec);

      RCLCPP_INFO(_node->get_logger(), "    positions:");
      for (const auto& pos : point.positions)
      {
        RCLCPP_INFO(_node->get_logger(), "      %f", pos);
      }

      RCLCPP_INFO(_node->get_logger(), "    velocities:");
      for (const auto& vel : point.velocities)
      {
        RCLCPP_INFO(_node->get_logger(), "      %f", vel);
      }

      // RCLCPP_INFO(_node->get_logger(), "    accelerations:");
      // for (const auto& accel : point.accelerations)
      // {
      //   RCLCPP_INFO(_node->get_logger(), "      %f", accel);
      // }

      // RCLCPP_INFO(_node->get_logger(), "    efforts:");
      // for (const auto& effort : point.effort)
      // {
      //   RCLCPP_INFO(_node->get_logger(), "      %f", effort);
      // }
    }
  }

  void TaskSolutionExecutor::print_multi_dof_joint_trajectory(const trajectory_msgs::msg::MultiDOFJointTrajectory& msg)
  {
    RCLCPP_INFO(_node->get_logger(), "Multi DOF Joint Trajectory:");

    RCLCPP_INFO(_node->get_logger(), "Header: ");
    RCLCPP_INFO(_node->get_logger(), "\tstamp: %f", msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec);
    RCLCPP_INFO(_node->get_logger(), "\tframe_id: %s", msg.header.frame_id.c_str());

    RCLCPP_INFO(_node->get_logger(), "Joint names:");
    for (const auto& joint_name : msg.joint_names)
    {
      RCLCPP_INFO(_node->get_logger(), "\t%s", joint_name.c_str());
    }

    RCLCPP_INFO(_node->get_logger(), "Points:");
    for (const auto& point : msg.points)
    {
      RCLCPP_INFO(_node->get_logger(),
                  "\tTime from start: %f",
                  point.time_from_start.sec + 1e-9 * point.time_from_start.nanosec);
      for (const auto& transform : point.transforms)
      {
        RCLCPP_INFO(_node->get_logger(), "\t\tTranslation:");
        RCLCPP_INFO(_node->get_logger(), "\t\t\tx: %f", transform.translation.x);
        RCLCPP_INFO(_node->get_logger(), "\t\t\ty: %f", transform.translation.y);
        RCLCPP_INFO(_node->get_logger(), "\t\t\tz: %f", transform.translation.z);
        RCLCPP_INFO(_node->get_logger(), "\t\tRotation:");
        RCLCPP_INFO(_node->get_logger(), "\t\t\tw: %f", transform.rotation.w);
        RCLCPP_INFO(_node->get_logger(), "\t\t\tx: %f", transform.rotation.x);
        RCLCPP_INFO(_node->get_logger(), "\t\t\ty: %f", transform.rotation.y);
        RCLCPP_INFO(_node->get_logger(), "\t\t\tz: %f", transform.rotation.z);
      }
      for (const auto& velocity : point.velocities)
      {
        RCLCPP_INFO(_node->get_logger(), "\t\tVelocity linear:");
        RCLCPP_INFO(_node->get_logger(), "\t\t\tx: %f", velocity.linear.x);
        RCLCPP_INFO(_node->get_logger(), "\t\t\ty: %f", velocity.linear.y);
        RCLCPP_INFO(_node->get_logger(), "\t\t\tz: %f", velocity.linear.z);
        RCLCPP_INFO(_node->get_logger(), "\t\tVelocity angular:");
        RCLCPP_INFO(_node->get_logger(), "\t\t\tx: %f", velocity.angular.x);
        RCLCPP_INFO(_node->get_logger(), "\t\t\ty: %f", velocity.angular.y);
        RCLCPP_INFO(_node->get_logger(), "\t\t\tz: %f", velocity.angular.z);
      }
    }
  }

}   // namespace gazebo_controller_manager