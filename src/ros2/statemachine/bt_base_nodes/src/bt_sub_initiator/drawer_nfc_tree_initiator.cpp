#include "bt_base_nodes/bt_sub_initiator/drawer_nfc_tree_initiator.hpp"

namespace bt_base_nodes
{
}

int main(int argc, char* argv[ ])
{
  rclcpp::init(argc, argv);
  std::map<std::string, communication_interfaces::msg::DrawerAddress> nfc_dictionary{{}};
  for (int i = 1; i <= 5; i++)
  {
    communication_interfaces::msg::DrawerAddress drawer_address;
    drawer_address.drawer_controller_id = 0;
    drawer_address.drawer_id = i;
    nfc_dictionary[std::to_string(i)] = drawer_address;
  }
  auto node = std::make_shared<bt_base_nodes::DrawerNFCTreeInitiator>(nfc_dictionary);
  node->configure();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}