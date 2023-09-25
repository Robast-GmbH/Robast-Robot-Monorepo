#include <gazebo_trajectory_executor/task_solution_executor.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  // creat ros2 node
  rclcpp::init(argc, argv);
  auto ros_node = std::make_shared<rclcpp::Node>("robot_trajectory_executor");

  const std::string ros_robot_trajectory_topic = "/door_opening_mechanism_controller/follow_joint_trajectory";
  const std::string task_execution_solution_action = "/execute_task_solution";

  auto robot_trajectory_executor = std::make_shared<gazebo_trajectory_executor::TaskSolutionExecutor>(
      ros_node, ros_robot_trajectory_topic, task_execution_solution_action);

  rclcpp::spin(ros_node);
  rclcpp::shutdown();
  return 0;
}
