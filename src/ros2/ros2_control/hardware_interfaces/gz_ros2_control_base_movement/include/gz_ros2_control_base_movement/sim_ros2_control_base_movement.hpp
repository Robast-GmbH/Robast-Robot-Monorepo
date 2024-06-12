#ifndef ROS2_CONTROL_BASE_MOVEMENT__GZ_SYSTEM_HPP_
#define ROS2_CONTROL_BASE_MOVEMENT__GZ_SYSTEM_HPP_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface_utils/hardware_interface_utils.hpp"
#include "hardware_interface_utils/prismatic_joint_state_monitor.hpp"
#include "nav_msgs/msg/odometry.hpp"
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

 private:
  std::vector<double> _hw_position_commands;
  std::vector<double> _hw_position_states;

  std::vector<double> _hw_velocity_commands;
  std::vector<double> _hw_velocity_states;

  std::string _LOGGER = "SimBaseMovement";

  std::shared_ptr<hardware_interface_utils::PrismaticJointStateMonitor> _prismatic_joint_state_monitor;
};

// Please mind:
// Usually, the implementation of the methods would be in a separate cpp file. But since this is a template class, the
// implementation has to be in the header file (at least this what copilot suggested).
// At least I could not make it work to put the implementation in a separate cpp file. It builds but I get a linking
// error everytime I launch it.

template <typename SimSystemInterface>
CallbackReturn SimBaseMovement<SimSystemInterface>::on_init(const hardware_interface::HardwareInfo& actuator_info)
{
  RCLCPP_INFO(rclcpp::get_logger(_LOGGER), "SimBaseMovement on_init()");

  if (hardware_interface::SystemInterface::on_init(actuator_info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  _hw_position_states.resize(SimSystemInterface::info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  _hw_position_commands.resize(SimSystemInterface::info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  _hw_velocity_states.resize(SimSystemInterface::info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  _hw_velocity_commands.resize(SimSystemInterface::info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  std::string odom_topic = SimSystemInterface::info_.hardware_parameters["odom_topic"];
  if (odom_topic.empty())
  {
    odom_topic = "/odom";
  }

  _prismatic_joint_state_monitor = std::make_shared<hardware_interface_utils::PrismaticJointStateMonitor>(odom_topic);

  return hardware_interface_utils::configure_joints(SimSystemInterface::info_.joints, _LOGGER);
}

template <typename SimSystemInterface>
CallbackReturn SimBaseMovement<SimSystemInterface>::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // reset values always when configuring hardware
  for (uint i = 0; i < _hw_position_states.size(); i++)
  {
    _hw_position_states[i] = 0;
    _hw_position_commands[i] = 0;
  }
  for (uint i = 0; i < _hw_velocity_states.size(); i++)
  {
    _hw_velocity_states[i] = 0;
    _hw_velocity_commands[i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger(_LOGGER), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

template <typename SimSystemInterface>
std::vector<hardware_interface::StateInterface> SimBaseMovement<SimSystemInterface>::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < SimSystemInterface::info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        SimSystemInterface::info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_hw_position_states[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        SimSystemInterface::info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_hw_velocity_states[i]));
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
        SimSystemInterface::info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_hw_position_commands[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        SimSystemInterface::info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_hw_velocity_commands[i]));
  }

  return command_interfaces;
}

template <typename SimSystemInterface>
CallbackReturn SimBaseMovement<SimSystemInterface>::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger(_LOGGER), "Activating ...please wait...");

  // command and state should be equal when starting
  for (uint i = 0; i < _hw_position_states.size(); i++)
  {
    _hw_position_commands[i] = _hw_position_states[i];
  }
  for (uint i = 0; i < _hw_velocity_states.size(); i++)
  {
    _hw_velocity_commands[i] = _hw_velocity_states[i];
  }

  RCLCPP_INFO(rclcpp::get_logger(_LOGGER), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

template <typename SimSystemInterface>
CallbackReturn SimBaseMovement<SimSystemInterface>::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  // TODO@Jacob: Check, if this will be triggered some day. Up to the point of working on this, I found no way that
  // TODO@Jacob: on_deactivate, on_cleanup, on_shutdown or on_error are triggered
  RCLCPP_INFO(rclcpp::get_logger(_LOGGER), "Deactivating ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger(_LOGGER), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

template <typename SimSystemInterface>
hardware_interface::return_type SimBaseMovement<SimSystemInterface>::read(const rclcpp::Time& /*time*/,
                                                                          const rclcpp::Duration& /*period*/)
{
  if (rclcpp::ok())
  {
    rclcpp::spin_some(_prismatic_joint_state_monitor);
  }

  _prismatic_joint_state_monitor->update_state(_hw_position_states, _hw_velocity_states);

  return hardware_interface::return_type::OK;
}

template <typename SimSystemInterface>
hardware_interface::return_type SimBaseMovement<SimSystemInterface>::write(const rclcpp::Time& /*time*/,
                                                                           const rclcpp::Duration& /*period*/)
{
  if (std::abs(_hw_velocity_commands[0]) > 0.00001)
  {
    // When we receive a velocity command, we know the trajectory execution has started, so the robot is in motion
    // and we want to track the position and velocity of the robot
    _prismatic_joint_state_monitor->set_trigger_trajectory_execution(true);
  }
  else
  {
    _prismatic_joint_state_monitor->set_trigger_trajectory_execution(false);
    _prismatic_joint_state_monitor->set_is_trajectory_execution_in_motion(false);
  }

  // We don't do anything with the commands here, because we have a chained controller that is responsible for taking
  // the command values of this joint and translating them into the commands for the wheels, so the cmd_vel topic
  return hardware_interface::return_type::OK;
}

#endif   // ROS2_CONTROL_BASE_MOVEMENT__GZ_SYSTEM_HPP_