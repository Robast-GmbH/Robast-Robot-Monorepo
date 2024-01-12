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
  hardware_interface::CallbackReturn DoorOpeningMechanismSystemHardware::on_init(
      const hardware_interface::HardwareInfo& info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    _ip_address = info_.hardware_parameters["ip_address"];
    _port = stod(info_.hardware_parameters["port"]);
    _direction = stod(info_.hardware_parameters["direction"]);
    _si_unit_factor = stod(info_.hardware_parameters["si_unit_factor"]);
    _is_prismatic_joint = info_.hardware_parameters["joint_type"] == "prismatic";

    _hw_position_states.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    _hw_position_commands.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    _hw_velocity_states.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    _hw_velocity_commands.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo& joint : info_.joints)
    {
      RCLCPP_INFO(
          rclcpp::get_logger("DoorOpeningMechanismSystemHardware"), "Configuring joint '%s'.", joint.name.c_str());

      // DoorOpeningMechanismSystemHardware has position and velocity state and command interface on each joint
      if (joint.command_interfaces.size() > 2)
      {
        RCLCPP_FATAL(rclcpp::get_logger("DoorOpeningMechanismSystemHardware"),
                     "Joint '%s' has %zu command interfaces found. 2 or less expected.",
                     joint.name.c_str(),
                     joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if ((joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) &&
          (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY))
      {
        RCLCPP_FATAL(rclcpp::get_logger("DoorOpeningMechanismSystemHardware"),
                     "Joint '%s' have %s command interfaces found. '%s' or '%s' expected.",
                     joint.name.c_str(),
                     joint.command_interfaces[0].name.c_str(),
                     hardware_interface::HW_IF_POSITION,
                     hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() > 2)
      {
        RCLCPP_FATAL(rclcpp::get_logger("DoorOpeningMechanismSystemHardware"),
                     "Joint '%s' has %zu state interface. 2 or less expected.",
                     joint.name.c_str(),
                     joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if ((joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) &&
          (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY))
      {
        RCLCPP_FATAL(rclcpp::get_logger("DoorOpeningMechanismSystemHardware"),
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

  hardware_interface::CallbackReturn DoorOpeningMechanismSystemHardware::on_configure(
      const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // Please mind! In order to get a connection to the dryve d1, there need to be a port forward on the router from the
    // port configured in DRYVE_D1_IP_ADDRESS_X_AXIS to the Port configured for Modbus TCP in the dryve d1 (usually 502)
    RCLCPP_INFO(rclcpp::get_logger("DoorOpeningMechanismSystemHardware"),
                "Configuring dryve d1 for ip address %s and port %d",
                _ip_address.c_str(),
                _port);
    _dryve_d1 =
        std::make_unique<dryve_d1_bridge::D1>(_ip_address, _port, std::make_unique<dryve_d1_bridge::SocketWrapper>());

    // Please mind! The state_machine only runs through if you made the following settings on the dryve d1:
    // (1) Drive Profile -> Modbus TCP Gateway
    // (2) Inputs/Outputs -> Enable
    // (3) Communication -> Port = 502
    // (4) Communication -> Modbus TCP Gateway ON
    // If these settings have already been made and once power was switched on and off you have to disable and
    // enable the Modbus TCP Gateway once in order to be able to connect to the dryve d1 again
    // TODO@Jacob: Find a possibility that Modbus TCP Gateway does not need to be switched on and off after power up
    _dryve_d1->run_dryve_state_machine();

    _dryve_d1->set_si_unit_factor(
        dryve_d1_bridge::map_si_unit_factor_double_to_char(_si_unit_factor, _is_prismatic_joint), _is_prismatic_joint);

    _dryve_d1->set_debug_mode_off();

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

    RCLCPP_INFO(rclcpp::get_logger("DoorOpeningMechanismSystemHardware"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> DoorOpeningMechanismSystemHardware::export_state_interfaces()
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

  std::vector<hardware_interface::CommandInterface> DoorOpeningMechanismSystemHardware::export_command_interfaces()
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

  hardware_interface::CallbackReturn DoorOpeningMechanismSystemHardware::on_activate(
      const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(rclcpp::get_logger("DoorOpeningMechanismSystemHardware"), "Activating ...please wait...");

    // command and state should be equal when starting
    for (uint i = 0; i < _hw_position_states.size(); i++)
    {
      _hw_position_commands[i] = _hw_position_states[i];
    }
    for (uint i = 0; i < _hw_velocity_states.size(); i++)
    {
      _hw_velocity_commands[i] = _hw_velocity_states[i];
    }

    RCLCPP_INFO(rclcpp::get_logger("DoorOpeningMechanismSystemHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DoorOpeningMechanismSystemHardware::on_deactivate(
      const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // TODO@Jacob: Check, if this will be triggered some day. Up to the point of working on this, I found no way that
    // TODO@Jacob: on_deactivate, on_cleanup, on_shutdown or on_error are triggered
    RCLCPP_INFO(rclcpp::get_logger("DoorOpeningMechanismSystemHardware"), "Deactivating ...please wait...");

    _dryve_d1->close_connection();

    RCLCPP_INFO(rclcpp::get_logger("DoorOpeningMechanismSystemHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DoorOpeningMechanismSystemHardware::on_shutdown(
      const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // TODO@Jacob: Check, if this will be triggered some day. Up to the point of working on this, I found no way that
    // TODO@Jacob: on_deactivate, on_cleanup, on_shutdown or on_error are triggered
    RCLCPP_INFO(rclcpp::get_logger("DoorOpeningMechanismSystemHardware"), "Shutting down ...please wait...");

    _dryve_d1->close_connection();

    RCLCPP_INFO(rclcpp::get_logger("DoorOpeningMechanismSystemHardware"), "Successful shutdown!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DoorOpeningMechanismSystemHardware::on_cleanup(
      const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // TODO@Jacob: Check, if this will be triggered some day. Up to the point of working on this, I found no way that
    // TODO@Jacob: on_deactivate, on_cleanup, on_shutdown or on_error are triggered
    RCLCPP_INFO(rclcpp::get_logger("DoorOpeningMechanismSystemHardware"), "Cleaning up ...please wait...");

    _dryve_d1->close_connection();

    RCLCPP_INFO(rclcpp::get_logger("DoorOpeningMechanismSystemHardware"), "Cleaned up successfully!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DoorOpeningMechanismSystemHardware::on_error(
      const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // TODO@Jacob: Check, if this will be triggered some day. Up to the point of working on this, I found no way that
    // TODO@Jacob: on_deactivate, on_cleanup, on_shutdown or on_error are triggered
    RCLCPP_INFO(rclcpp::get_logger("DoorOpeningMechanismSystemHardware"), "On Error: Cleaning up ...please wait...");

    _dryve_d1->close_connection();

    RCLCPP_INFO(rclcpp::get_logger("DoorOpeningMechanismSystemHardware"), "On Error: Cleaned up successfully!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type DoorOpeningMechanismSystemHardware::read(const rclcpp::Time& /*time*/,
                                                                           const rclcpp::Duration& /*period*/)
  {
    // Please mind:
    // Setting the _si_unit_factor to the dryve d1 works in principle. If you check the displayed position values in the
    // web interface, the values change when you change the si_unit_factor.
    // BUT: No matter what si_unit_factor you set for the dryve d1, we get the same values from the tcp connection!
    // Therefore we have to divide and multiply the si_unit_factor by hand in order to get the correct values
    double hw_position_state =
        static_cast<double>(_dryve_d1->read_object_value(_dryve_d1->OBJECT_INDEX_1_READ_POSITION_ACTUAL_VALUE,
                                                         _dryve_d1->OBJECT_INDEX_2_READ_POSITION_ACTUAL_VALUE)) /
        _si_unit_factor * _direction;

    double hw_velocity_states =
        static_cast<double>(_dryve_d1->read_object_value(_dryve_d1->OBJECT_INDEX_1_READ_VELOCITY_ACTUAL_VALUE,
                                                         _dryve_d1->OBJECT_INDEX_2_READ_VELOCITY_ACTUAL_VALUE)) /
        _si_unit_factor * _direction;

    if (_is_prismatic_joint)
    {
      _hw_position_states[0] = dryve_d1_bridge::mm_to_m(hw_position_state);
      _hw_velocity_states[0] = dryve_d1_bridge::mm_to_m(hw_velocity_states);
    }
    else
    {
      _hw_position_states[0] = dryve_d1_bridge::degree_to_rad(hw_position_state);
      _hw_velocity_states[0] = dryve_d1_bridge::degree_to_rad(hw_velocity_states);
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type DoorOpeningMechanismSystemHardware::write(const rclcpp::Time& /*time*/,
                                                                            const rclcpp::Duration& /*period*/)
  {
    _dryve_d1->set_profile_velocity(_hw_velocity_commands[0] * _si_unit_factor * _direction,
                                    dryve_d1_bridge::ACCELERATION,
                                    dryve_d1_bridge::DECELERATION);

    return hardware_interface::return_type::OK;
  }

}   // namespace ros2_control_plugin_door_opening_mechanism

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ros2_control_plugin_door_opening_mechanism::DoorOpeningMechanismSystemHardware,
                       hardware_interface::SystemInterface)
