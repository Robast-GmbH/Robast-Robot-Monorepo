#include "gz_ros2_control_base_movement/ign_ros2_control_base_movement.hpp"

namespace ign_ros2_control_base_movement
{
  bool IgnBaseMovementSystemHardware::initSim(rclcpp::Node::SharedPtr &model_nh,
                                              std::map<std::string, ignition::gazebo::Entity> &enableJoints,
                                              const hardware_interface::HardwareInfo &hardware_info,
                                              ignition::gazebo::EntityComponentManager &_ecm,
                                              int &update_rate)
  {
    return true;
  }

}   // namespace ign_ros2_control_base_movement

#include "pluginlib/class_list_macros.hpp"   // NOLINT
PLUGINLIB_EXPORT_CLASS(ign_ros2_control_base_movement::IgnBaseMovementSystemHardware,
                       ign_ros2_control::IgnitionSystemInterface)
