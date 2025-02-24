#include "door_opening_mechanism_mtc/door_opening_mechanism_mtc.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(
    true);   // Not having this caused my planner to give zero time to all trajectory points - so cartesian planners
             // work but OMPL plannners do not.
  auto door_opening_mechanism_mtc_node = std::make_shared<door_opening_mechanism_mtc::DoorMechanismMtc>(options);

  rclcpp::spin(door_opening_mechanism_mtc_node);

  rclcpp::shutdown();

  return 0;
}