#include "door_opening_mechanism_mtc/door_opening_mechanism_mtc.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto door_opening_mechanism_mtc_node = std::make_shared<door_opening_mechanism_mtc::DoorMechanismMtc>();

  rclcpp::spin(door_opening_mechanism_mtc_node);

  rclcpp::shutdown();

  return 0;
}