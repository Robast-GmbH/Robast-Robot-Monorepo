#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "drawer_sm/heartbeat_tree_spawner.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<drawer_sm::HeartbeatTreeSpawner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
