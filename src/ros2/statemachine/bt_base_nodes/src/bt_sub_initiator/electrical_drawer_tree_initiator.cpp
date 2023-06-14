#include "bt_base_nodes/bt_sub_initiator/drawer_tree_initiator.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bt_base_nodes::DrawerTreeInitiator>(
        "trigger_electric_drawer_tree");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}