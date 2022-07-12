#include <memory>

#include "robast_drawer_sym/drawer_sym.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robast::DrawerSym>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
