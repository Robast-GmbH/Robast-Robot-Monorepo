// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gz_ros2_control_base_movement/gz_ros2_control_base_movement.hpp"

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#ifdef GZ_HEADERS
#include <gz/msgs/imu.pb.h>

#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/JointForce.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/transport/Node.hh>
#define GZ_TRANSPORT_NAMESPACE gz::transport::
#define GZ_MSGS_NAMESPACE      gz::msgs::
#else
#include <ignition/msgs/imu.pb.h>

#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/Imu.hh>
#include <ignition/gazebo/components/JointForce.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/LinearAcceleration.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Sensor.hh>
#include <ignition/transport/Node.hh>
#define GZ_TRANSPORT_NAMESPACE ignition::transport::
#define GZ_MSGS_NAMESPACE      ignition::msgs::
#endif

#include <hardware_interface/hardware_info.hpp>

namespace gz_ros2_control_base_movement
{

  bool BaseMovementSystemHardware::initSim(rclcpp::Node::SharedPtr &model_nh,
                                           std::map<std::string, sim::Entity> &enableJoints,
                                           const hardware_interface::HardwareInfo &hardware_info,
                                           sim::EntityComponentManager &_ecm,
                                           int &update_rate)
  {
    return true;
  }

  CallbackReturn BaseMovementSystemHardware::on_init(const hardware_interface::HardwareInfo &actuator_info)
  {
    RCLCPP_INFO(rclcpp::get_logger("BaseMovementSystemHardware"), "BaseMovementSystemHardware on_init()");

    if (hardware_interface::SystemInterface::on_init(actuator_info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    _hw_position_states.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    _hw_position_commands.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    _hw_velocity_states.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    _hw_velocity_commands.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      RCLCPP_INFO(rclcpp::get_logger("BaseMovementSystemHardware"), "Configuring joint '%s'.", joint.name.c_str());

      // BaseMovementSystemHardware has position and velocity state and command interface on each joint
      if (joint.command_interfaces.size() > 2)
      {
        RCLCPP_FATAL(rclcpp::get_logger("BaseMovementSystemHardware"),
                     "Joint '%s' has %zu command interfaces found. 2 or less expected.",
                     joint.name.c_str(),
                     joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if ((joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) &&
          (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY))
      {
        RCLCPP_FATAL(rclcpp::get_logger("BaseMovementSystemHardware"),
                     "Joint '%s' have %s command interfaces found. '%s' or '%s' expected.",
                     joint.name.c_str(),
                     joint.command_interfaces[0].name.c_str(),
                     hardware_interface::HW_IF_POSITION,
                     hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() > 2)
      {
        RCLCPP_FATAL(rclcpp::get_logger("BaseMovementSystemHardware"),
                     "Joint '%s' has %zu state interface. 2 or less expected.",
                     joint.name.c_str(),
                     joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if ((joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) &&
          (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY))
      {
        RCLCPP_FATAL(rclcpp::get_logger("BaseMovementSystemHardware"),
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

  CallbackReturn BaseMovementSystemHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
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

    RCLCPP_INFO(rclcpp::get_logger("BaseMovementSystemHardware"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> BaseMovementSystemHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_hw_position_states[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_hw_velocity_states[i]));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> BaseMovementSystemHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_hw_position_commands[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_hw_velocity_commands[i]));
    }

    return command_interfaces;
  }

  CallbackReturn BaseMovementSystemHardware::on_activate(const rclcpp_lifecycle::State &previous_state)
  {
    RCLCPP_INFO(rclcpp::get_logger("BaseMovementSystemHardware"), "Activating ...please wait...");

    // command and state should be equal when starting
    for (uint i = 0; i < _hw_position_states.size(); i++)
    {
      _hw_position_commands[i] = _hw_position_states[i];
    }
    for (uint i = 0; i < _hw_velocity_states.size(); i++)
    {
      _hw_velocity_commands[i] = _hw_velocity_states[i];
    }

    RCLCPP_INFO(rclcpp::get_logger("BaseMovementSystemHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  CallbackReturn BaseMovementSystemHardware::on_deactivate(const rclcpp_lifecycle::State &previous_state)
  {
    // TODO@Jacob: Check, if this will be triggered some day. Up to the point of working on this, I found no way that
    // TODO@Jacob: on_deactivate, on_cleanup, on_shutdown or on_error are triggered
    RCLCPP_INFO(rclcpp::get_logger("BaseMovementSystemHardware"), "Deactivating ...please wait...");

    RCLCPP_INFO(rclcpp::get_logger("BaseMovementSystemHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type BaseMovementSystemHardware::read(const rclcpp::Time & /*time*/,
                                                                   const rclcpp::Duration & /*period*/)
  {
    // TODO@Jacob: Implement reading sensors

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type BaseMovementSystemHardware::write(const rclcpp::Time & /*time*/,
                                                                    const rclcpp::Duration & /*period*/)
  {
    // TODO@Jacob: Implement writing to actuators

    return hardware_interface::return_type::OK;
  }
}   // namespace gz_ros2_control_base_movement

#include "pluginlib/class_list_macros.hpp"   // NOLINT
PLUGINLIB_EXPORT_CLASS(gz_ros2_control_base_movement::BaseMovementSystemHardware,
                       gz_ros2_control::GazeboSimSystemInterface)
