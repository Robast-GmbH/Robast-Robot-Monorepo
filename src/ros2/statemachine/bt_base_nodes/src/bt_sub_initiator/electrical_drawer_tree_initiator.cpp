#include "bt_base_nodes/bt_sub_initiator/electrical_drawer_tree_initiator.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bt_base_nodes::ElectricalDrawerTreeInitiator>();
    node->configure("trigger_electric_drawer_tree", "default_electrical_drawer_tree");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}