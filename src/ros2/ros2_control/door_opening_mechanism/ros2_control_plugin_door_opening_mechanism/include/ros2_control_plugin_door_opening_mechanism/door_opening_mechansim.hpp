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

#ifndef ROS2_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM__RRBOT_HPP_
#define ROS2_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM__RRBOT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "dryve_d1_bridge/d1.hpp"
#include "dryve_d1_bridge/d1_configs.hpp"
#include "dryve_d1_bridge/socket_wrapper.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2_control_plugin_door_opening_mechanism/visibility_control.h"

namespace ros2_control_plugin_door_opening_mechanism
{
  class DoorOpeningMechanismSystemPositionOnlyHardware : public hardware_interface::SystemInterface
  {
   public:
    RCLCPP_SHARED_PTR_DEFINITIONS(DoorOpeningMechanismSystemPositionOnlyHardware)

    ROS2_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM_PUBLIC hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo& info) override;

    ROS2_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM_PUBLIC hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM_PUBLIC std::vector<hardware_interface::StateInterface>
    export_state_interfaces() override;

    ROS2_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM_PUBLIC std::vector<hardware_interface::CommandInterface>
    export_command_interfaces() override;

    ROS2_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM_PUBLIC hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM_PUBLIC hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM_PUBLIC hardware_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM_PUBLIC hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM_PUBLIC hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM_PUBLIC hardware_interface::return_type read(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

    ROS2_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM_PUBLIC hardware_interface::return_type write(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

   private:
    // Parameters for the RRBot simulation
    double hw_start_sec_;
    double hw_stop_sec_;
    double hw_slowdown_;

    std::vector<double> _hw_position_commands;
    std::vector<double> _hw_position_states;

    std::vector<double> _hw_velocity_commands;
    std::vector<double> _hw_velocity_states;

    // Please mind: Actually you could get this value from the d1_dryve via get_si_unit_factor(), but I get stupid
    // values from it
    const double _X_AXIS_SI_UNIT_FACTOR = -100000;

    const double _X_AXIS_VELOCITY = 10;
    const double _X_AXIS_ACCELERATION = 10;
    const double _X_AXIS_DECELERATION = 10;

    const double _Y_AXIS_SI_UNIT_FACTOR = 100000;

    const double _Y_AXIS_VELOCITY = 10;
    const double _Y_AXIS_ACCELERATION = 10;
    const double _Y_AXIS_DECELERATION = 10;

    std::unique_ptr<dryve_d1_bridge::D1> _x_axis;
    std::unique_ptr<dryve_d1_bridge::D1> _y_axis;
  };

}   // namespace ros2_control_plugin_door_opening_mechanism

#endif   // ROS2_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM__RRBOT_HPP_
