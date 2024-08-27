#include "gz_ros2_control_base_movement/gz_ros2_control_base_movement.hpp"

namespace gz_ros2_control_base_movement
{

  bool GzBaseMovementSystemHardware::initSim(rclcpp::Node::SharedPtr &model_nh,
                                             std::map<std::string, sim::Entity> &joints,
                                             const hardware_interface::HardwareInfo &hardware_info,
                                             sim::EntityComponentManager &_ecm,
                                             unsigned int update_rate)
  {
    return true;
  }

}   // namespace gz_ros2_control_base_movement

#include "pluginlib/class_list_macros.hpp"   // NOLINT
PLUGINLIB_EXPORT_CLASS(gz_ros2_control_base_movement::GzBaseMovementSystemHardware,
                       gz_ros2_control::GazeboSimSystemInterface)
