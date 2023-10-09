// Copyright 2020 ros2_control Development Team
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

#include "ros2_control_plugin_door_opening_mechanism/door_opening_mechansim.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_plugin_door_opening_mechanism
{
  hardware_interface::CallbackReturn DoorOpeningMechanismSystemPositionOnlyHardware::on_init(
    const hardware_interface::HardwareInfo& info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
    hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
    hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
    // END: This part here is for exemplary purposes - Please do not copy to your production code
    _hw_position_states.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    _hw_position_commands.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    _hw_velocity_states.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    _hw_velocity_commands.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo& joint : info_.joints)
    {
      // DoorOpeningMechanismSystemPositionOnlyHardware has position and velocity state and command interface on each
      // joint
      if (joint.command_interfaces.size() > 2)
      {
        RCLCPP_FATAL(rclcpp::get_logger("DoorOpeningMechanismSystemPositionOnlyHardware"),
                     "Joint '%s' has %zu command interfaces found. 2 or less expected.",
                     joint.name.c_str(),
                     joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if ((joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) &&
          (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY))
      {
        RCLCPP_FATAL(rclcpp::get_logger("DoorOpeningMechanismSystemPositionOnlyHardware"),
                     "Joint '%s' have %s command interfaces found. '%s' or '%s' expected.",
                     joint.name.c_str(),
                     joint.command_interfaces[0].name.c_str(),
                     hardware_interface::HW_IF_POSITION,
                     hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() > 2)
      {
        RCLCPP_FATAL(rclcpp::get_logger("DoorOpeningMechanismSystemPositionOnlyHardware"),
                     "Joint '%s' has %zu state interface. 2 or less expected.",
                     joint.name.c_str(),
                     joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if ((joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) &&
          (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY))
      {
        RCLCPP_FATAL(rclcpp::get_logger("DoorOpeningMechanismSystemPositionOnlyHardware"),
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

  hardware_interface::CallbackReturn DoorOpeningMechanismSystemPositionOnlyHardware::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // Please mind! In order to get a connection to the dryve d1, there need to be a port forward on the router from the
    // port configured in DRYVE_D1_IP_ADDRESS_X_AXIS to the Port configured for Modbus TCP in the dryve d1 (usually 502)
    RCLCPP_INFO(rclcpp::get_logger("DoorOpeningMechanismSystemPositionOnlyHardware"), "Configuring x-axis....");
    _x_axis = std::make_unique<dryve_d1_bridge::D1>(dryve_d1_bridge::DRYVE_D1_IP_ADDRESS_X_AXIS,
                                                    dryve_d1_bridge::PORT_X_AXIS,
                                                    std::make_unique<dryve_d1_bridge::SocketWrapper>());
    RCLCPP_INFO(rclcpp::get_logger("DoorOpeningMechanismSystemPositionOnlyHardware"), "Configuring y-axis....");
    _y_axis = std::make_unique<dryve_d1_bridge::D1>(dryve_d1_bridge::DRYVE_D1_IP_ADDRESS_Y_AXIS,
                                                    dryve_d1_bridge::PORT_Y_AXIS,
                                                    std::make_unique<dryve_d1_bridge::SocketWrapper>());

    // Please mind! The state_machine only runs through if you made the following settings on the dryve d1:
    // (1) Drive Profile -> Modbus TCP Gateway
    // (2) Inputs/Outputs -> Enable
    // (3) Communication -> Port = 502
    // (4) Communication -> Modbus TCP Gateway ON
    _x_axis->run_dryve_state_machine();
    _y_axis->run_dryve_state_machine();
    _x_axis->set_debug_mode_off();
    _y_axis->set_debug_mode_off();

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

    RCLCPP_INFO(rclcpp::get_logger("DoorOpeningMechanismSystemPositionOnlyHardware"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface>
  DoorOpeningMechanismSystemPositionOnlyHardware::export_state_interfaces()
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

  std::vector<hardware_interface::CommandInterface>
  DoorOpeningMechanismSystemPositionOnlyHardware::export_command_interfaces()
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

  hardware_interface::CallbackReturn DoorOpeningMechanismSystemPositionOnlyHardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(rclcpp::get_logger("DoorOpeningMechanismSystemPositionOnlyHardware"), "Activating ...please wait...");

    for (int i = 0; i < hw_start_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(rclcpp::get_logger("DoorOpeningMechanismSystemPositionOnlyHardware"),
                  "%.1f seconds left...",
                  hw_start_sec_ - i);
    }
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    // command and state should be equal when starting
    for (uint i = 0; i < _hw_position_states.size(); i++)
    {
      _hw_position_commands[i] = _hw_position_states[i];
    }
    for (uint i = 0; i < _hw_velocity_states.size(); i++)
    {
      _hw_velocity_commands[i] = _hw_velocity_states[i];
    }

    RCLCPP_INFO(rclcpp::get_logger("DoorOpeningMechanismSystemPositionOnlyHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DoorOpeningMechanismSystemPositionOnlyHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // TODO@Jacob: Check, if this will be triggered some day. Up to the point of working on this, I found no way that
    // TODO@Jacob: on_deactivate, on_cleanup, on_shutdown or on_error are triggered

    // dryve D1 to shutdown state

    RCLCPP_INFO(rclcpp::get_logger("DoorOpeningMechanismSystemPositionOnlyHardware"), "Deactivating ...please wait...");

    _x_axis->close_connection();
    _y_axis->close_connection();

    RCLCPP_INFO(rclcpp::get_logger("DoorOpeningMechanismSystemPositionOnlyHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DoorOpeningMechanismSystemPositionOnlyHardware::on_shutdown(
    const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // TODO@Jacob: Check, if this will be triggered some day. Up to the point of working on this, I found no way that
    // TODO@Jacob: on_deactivate, on_cleanup, on_shutdown or on_error are triggered

    // dryve D1 to shutdown state

    RCLCPP_INFO(rclcpp::get_logger("DoorOpeningMechanismSystemPositionOnlyHardware"),
                "Shutting down ...please wait...");

    _x_axis->close_connection();
    _y_axis->close_connection();

    RCLCPP_INFO(rclcpp::get_logger("DoorOpeningMechanismSystemPositionOnlyHardware"), "Successful shutdown!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DoorOpeningMechanismSystemPositionOnlyHardware::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // TODO@Jacob: Check, if this will be triggered some day. Up to the point of working on this, I found no way that
    // TODO@Jacob: on_deactivate, on_cleanup, on_shutdown or on_error are triggered

    // dryve D1 to shutdown state

    RCLCPP_INFO(rclcpp::get_logger("DoorOpeningMechanismSystemPositionOnlyHardware"), "Cleaning up ...please wait...");

    _x_axis->close_connection();
    _y_axis->close_connection();

    RCLCPP_INFO(rclcpp::get_logger("DoorOpeningMechanismSystemPositionOnlyHardware"), "Cleaned up successfully!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DoorOpeningMechanismSystemPositionOnlyHardware::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // TODO@Jacob: Check, if this will be triggered some day. Up to the point of working on this, I found no way that
    // TODO@Jacob: on_deactivate, on_cleanup, on_shutdown or on_error are triggered

    // dryve D1 to shutdown state

    RCLCPP_INFO(rclcpp::get_logger("DoorOpeningMechanismSystemPositionOnlyHardware"),
                "On Error: Cleaning up ...please wait...");

    _x_axis->close_connection();
    _y_axis->close_connection();

    RCLCPP_INFO(rclcpp::get_logger("DoorOpeningMechanismSystemPositionOnlyHardware"),
                "On Error: Cleaned up successfully!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type DoorOpeningMechanismSystemPositionOnlyHardware::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
  {
    _hw_position_states[1] =
      static_cast<double>(_x_axis->read_object_value(_x_axis->OBJECT_INDEX_1_READ_POSITION_ACTUAL_VALUE,
                                                     _x_axis->OBJECT_INDEX_2_READ_POSITION_ACTUAL_VALUE)) /
      dryve_d1_bridge::X_AXIS_SI_UNIT_FACTOR;
    _hw_velocity_states[1] =
      static_cast<double>(_x_axis->read_object_value(_x_axis->OBJECT_INDEX_1_READ_VELOCITY_ACTUAL_VALUE,
                                                     _x_axis->OBJECT_INDEX_2_READ_VELOCITY_ACTUAL_VALUE)) /
      dryve_d1_bridge::X_AXIS_SI_UNIT_FACTOR;

    _hw_position_states[0] =
      static_cast<double>(_y_axis->read_object_value(_y_axis->OBJECT_INDEX_1_READ_POSITION_ACTUAL_VALUE,
                                                     _y_axis->OBJECT_INDEX_2_READ_POSITION_ACTUAL_VALUE)) /
      dryve_d1_bridge::Y_AXIS_SI_UNIT_FACTOR;
    _hw_velocity_states[0] =
      static_cast<double>(_y_axis->read_object_value(_y_axis->OBJECT_INDEX_1_READ_VELOCITY_ACTUAL_VALUE,
                                                     _y_axis->OBJECT_INDEX_2_READ_VELOCITY_ACTUAL_VALUE)) /
      dryve_d1_bridge::Y_AXIS_SI_UNIT_FACTOR;

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type DoorOpeningMechanismSystemPositionOnlyHardware::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
  {
    _x_axis->set_profile_velocity(
      _hw_velocity_commands[1], dryve_d1_bridge::X_AXIS_ACCELERATION, dryve_d1_bridge::X_AXIS_DECELERATION);
    _y_axis->set_profile_velocity(
      _hw_velocity_commands[0], dryve_d1_bridge::Y_AXIS_ACCELERATION, dryve_d1_bridge::Y_AXIS_DECELERATION);

    return hardware_interface::return_type::OK;
  }

}   // namespace ros2_control_plugin_door_opening_mechanism

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ros2_control_plugin_door_opening_mechanism::DoorOpeningMechanismSystemPositionOnlyHardware,
                       hardware_interface::SystemInterface)
