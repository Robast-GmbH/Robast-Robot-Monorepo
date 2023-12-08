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

#ifndef GZ_ROS2_CONTROL_BASE_MOVEMENT__GZ_SYSTEM_HPP_
#define GZ_ROS2_CONTROL_BASE_MOVEMENT__GZ_SYSTEM_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "gz_ros2_control/gz_system_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2_control_base_movement/ros2_control_base_movement.hpp"

namespace gz_ros2_control_base_movement
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  // These class must inherit `gz_ros2_control::GazeboSimSystemInterface` which implements a
  // simulated `ros2_control` `hardware_interface::SystemInterface`.

  class BaseMovementSystemHardware : public gz_ros2_control::GazeboSimSystemInterface
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

    // Documentation Inherited
    bool initSim(rclcpp::Node::SharedPtr& model_nh,
                 std::map<std::string, sim::Entity>& joints,
                 const hardware_interface::HardwareInfo& hardware_info,
                 sim::EntityComponentManager& _ecm,
                 int& update_rate) override;

   private:
    std::vector<double> _hw_position_commands;
    std::vector<double> _hw_position_states;

    std::vector<double> _hw_velocity_commands;
    std::vector<double> _hw_velocity_states;
  };

}   // namespace gz_ros2_control_base_movement

#endif   // GZ_ROS2_CONTROL_BASE_MOVEMENT__GZ_SYSTEM_HPP_
