#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robot_client/robot_client.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rmf_robot_client::RobotClient>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
