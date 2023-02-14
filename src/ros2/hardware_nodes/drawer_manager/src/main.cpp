#include <memory>

#include "drawer_manager/drawer_manager.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nfc_gate::DrawerManager>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}