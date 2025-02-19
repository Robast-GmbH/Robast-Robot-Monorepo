#include "qr_code_scanner.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<QrCodeScanner>();
  rclcpp::executors::MultiThreadedExecutor executor;
  node->for_each_callback_group(
    [&executor, &node](rclcpp::CallbackGroup::SharedPtr group)
    {
      executor.add_callback_group(group, node->get_node_base_interface(), true);
    });
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}