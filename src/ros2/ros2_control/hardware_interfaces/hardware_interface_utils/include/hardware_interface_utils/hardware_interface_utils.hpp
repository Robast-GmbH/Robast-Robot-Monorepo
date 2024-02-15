#ifndef HARDWARE_INTERFACE_UTILS__HARDWARE_INTERFACE_UTILS_HPP_
#define HARDWARE_INTERFACE_UTILS__HARDWARE_INTERFACE_UTILS_HPP_

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace hardware_interface_utils
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn configure_joints(const std::vector<hardware_interface::ComponentInfo>& joints, std::string logger);

}   // namespace hardware_interface_utils

#endif   // HARDWARE_INTERFACE_UTILS__HARDWARE_INTERFACE_UTILS_HPP_