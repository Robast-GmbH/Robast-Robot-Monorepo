#include "bt_base_nodes/bt_sub_initiator/heartbeat_tree_initiator.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bt_base_nodes::HeartbeatTreeInitiator>();
  node->configure();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}