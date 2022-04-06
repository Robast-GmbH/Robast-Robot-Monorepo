#include <memory>

#include "robast_drawer_node/drawer_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robast_drawer_node::DrawerNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
