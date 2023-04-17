#include <memory>

#include "test/test_nfc_bridge.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<nfc_bridge::TestNFCBridge>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}