#include <rclcpp/rclcpp.hpp>

#include "gazebo_trajectory_executor/joint_trajectory_executor.hpp"

int main(int argc, char* argv[])
{
  // creat ros2 node
  rclcpp::init(argc, argv);

  // create controller
  auto joint_trajectory_executor = std::make_shared<gazebo_trajectory_executor::JointTrajectoryExecutor>();
  // run node until it's exited
  rclcpp::spin(joint_trajectory_executor);
  // clean up
  rclcpp::shutdown();
  return 0;
}