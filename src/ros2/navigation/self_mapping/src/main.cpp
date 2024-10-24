#include "self_mapping/nav_action.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavAction>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}