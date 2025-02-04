#include "qr_code_scanner.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<QrCodeScanner>();
  rclcpp::executors::MultiThreadedExecutor executor;

  // Add the node to the executor
  executor.add_node(node);

  // Spin the executor in a separate thread
  executor.spin();

  rclcpp::shutdown();
  return 0;
}