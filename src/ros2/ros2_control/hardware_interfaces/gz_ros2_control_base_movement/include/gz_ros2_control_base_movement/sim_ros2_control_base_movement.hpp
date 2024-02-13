#ifndef ROS2_CONTROL_BASE_MOVEMENT__GZ_SYSTEM_HPP_
#define ROS2_CONTROL_BASE_MOVEMENT__GZ_SYSTEM_HPP_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// base_movement.hpp
template <typename SimSystemInterface>
class SimBaseMovement : public SimSystemInterface
{
 public:
  // Documentation Inherited
  CallbackReturn on_init(const hardware_interface::HardwareInfo& system_info) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  // Documentation Inherited
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // Documentation Inherited
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Documentation Inherited
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  // Documentation Inherited
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  // Documentation Inherited
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  // Documentation Inherited
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 protected:
  std::vector<double> hw_position_commands_;
  std::vector<double> hw_position_states_;

  std::vector<double> hw_velocity_commands_;
  std::vector<double> hw_velocity_states_;
};

// Please mind:
// Usually, the implementation of the methods would be in a separate cpp file. But since this is a template class, the
// implementation has to be in the header file (at least this what copilot suggested).
// At least I could not make it work to put the implementation in a separate cpp file. It builds but I get a linking
// error everytime I launch it.

template <typename SimSystemInterface>
CallbackReturn SimBaseMovement<SimSystemInterface>::on_init(const hardware_interface::HardwareInfo& actuator_info)
{
  RCLCPP_INFO(rclcpp::get_logger("SimBaseMovement"), "SimBaseMovement on_init()");

  if (hardware_interface::SystemInterface::on_init(actuator_info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_position_states_.resize(SimSystemInterface::info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_position_commands_.resize(SimSystemInterface::info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocity_states_.resize(SimSystemInterface::info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocity_commands_.resize(SimSystemInterface::info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo& joint : SimSystemInterface::info_.joints)
  {
    RCLCPP_INFO(rclcpp::get_logger("SimBaseMovement"), "Configuring joint '%s'.", joint.name.c_str());

    // SimBaseMovement has position and velocity state and command interface on each joint
    if (joint.command_interfaces.size() > 2)
    {
      RCLCPP_FATAL(rclcpp::get_logger("SimBaseMovement"),
                   "Joint '%s' has %zu command interfaces found. 2 or less expected.",
                   joint.name.c_str(),
                   joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if ((joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) &&
        (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_FATAL(rclcpp::get_logger("SimBaseMovement"),
                   "Joint '%s' have %s command interfaces found. '%s' or '%s' expected.",
                   joint.name.c_str(),
                   joint.command_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION,
                   hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() > 2)
    {
      RCLCPP_FATAL(rclcpp::get_logger("SimBaseMovement"),
                   "Joint '%s' has %zu state interface. 2 or less expected.",
                   joint.name.c_str(),
                   joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if ((joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) &&
        (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_FATAL(rclcpp::get_logger("SimBaseMovement"),
                   "Joint '%s' have %s state interface. '%s' or '%s' expected.",
                   joint.name.c_str(),
                   joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION,
                   hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

template <typename SimSystemInterface>
CallbackReturn SimBaseMovement<SimSystemInterface>::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // reset values always when configuring hardware
  for (uint i = 0; i < hw_position_states_.size(); i++)
  {
    hw_position_states_[i] = 0;
    hw_position_commands_[i] = 0;
  }
  for (uint i = 0; i < hw_velocity_states_.size(); i++)
  {
    hw_velocity_states_[i] = 0;
    hw_velocity_commands_[i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("SimBaseMovement"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

template <typename SimSystemInterface>
std::vector<hardware_interface::StateInterface> SimBaseMovement<SimSystemInterface>::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < SimSystemInterface::info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        SimSystemInterface::info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        SimSystemInterface::info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_states_[i]));
  }

  return state_interfaces;
}

template <typename SimSystemInterface>
std::vector<hardware_interface::CommandInterface> SimBaseMovement<SimSystemInterface>::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < SimSystemInterface::info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        SimSystemInterface::info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        SimSystemInterface::info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_commands_[i]));
  }

  return command_interfaces;
}

template <typename SimSystemInterface>
CallbackReturn SimBaseMovement<SimSystemInterface>::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("SimBaseMovement"), "Activating ...please wait...");

  // command and state should be equal when starting
  for (uint i = 0; i < hw_position_states_.size(); i++)
  {
    hw_position_commands_[i] = hw_position_states_[i];
  }
  for (uint i = 0; i < hw_velocity_states_.size(); i++)
  {
    hw_velocity_commands_[i] = hw_velocity_states_[i];
  }

  RCLCPP_INFO(rclcpp::get_logger("SimBaseMovement"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

template <typename SimSystemInterface>
CallbackReturn SimBaseMovement<SimSystemInterface>::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  // TODO@Jacob: Check, if this will be triggered some day. Up to the point of working on this, I found no way that
  // TODO@Jacob: on_deactivate, on_cleanup, on_shutdown or on_error are triggered
  RCLCPP_INFO(rclcpp::get_logger("SimBaseMovement"), "Deactivating ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger("SimBaseMovement"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

template <typename SimSystemInterface>
hardware_interface::return_type SimBaseMovement<SimSystemInterface>::read(const rclcpp::Time& /*time*/,
                                                                          const rclcpp::Duration& /*period*/)
{
  // Right now, this is kind of a mocked implementation. I am not sure if we need the feedback for the
  // mobile_base joint. It might be enough that we have this joint for planning and executing the base movement open
  // loop. If we need the feedback, we need to implement this here.
  return hardware_interface::return_type::OK;
}

template <typename SimSystemInterface>
hardware_interface::return_type SimBaseMovement<SimSystemInterface>::write(const rclcpp::Time& /*time*/,
                                                                           const rclcpp::Duration& /*period*/)
{
  // We don't do anything with the commands here, because we have a chained controller that is responsible for taking
  // the command values of this joint and translating them into the commands for the wheels, so the cmd_vel topic
  return hardware_interface::return_type::OK;
}

#endif   // ROS2_CONTROL_BASE_MOVEMENT__GZ_SYSTEM_HPP_