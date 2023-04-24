#include <gazebo_controller_manager/robot_trajectory_executor.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  // creat ros2 node
  rclcpp::init(argc, argv);
  auto ros_node = std::make_shared<rclcpp::Node>("robot_trajectory_executor");
  // variable
  std::vector<std::string> joint_names;
  std::vector<std::string> default_joint_names = {
      "drawer_1_joint", "drawer_2_joint", "drawer_3_joint", "drawer_4_joint", "drawer_5_joint"};
  // parameters
  ros_node->declare_parameter("joint_names", default_joint_names);
  joint_names = ros_node->get_parameter("joint_names").get_parameter_value().get<std::vector<std::string>>();

  auto robot_trajectory_executor = std::make_shared<gazebo_controller_manager::RobotTrajectoryExecutor>(
      ros_node, joint_names, "/door_opening_mechanism_controller/follow_joint_trajectory");

  rclcpp::spin(ros_node);
  rclcpp::shutdown();
  return 0;
}
