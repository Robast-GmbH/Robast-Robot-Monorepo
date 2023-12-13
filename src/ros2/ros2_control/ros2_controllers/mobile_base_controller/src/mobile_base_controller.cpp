// Copyright (c) 2023, PAL Robotics
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

#include "mobile_base_controller/mobile_base_controller.hpp"

#include "controller_interface/helpers.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mobile_base_controller
{

  controller_interface::CallbackReturn MobileBaseController::on_init()
  {
    RCLCPP_INFO(this->get_node()->get_logger(), "starting on_init()");

    try
    {
      param_listener_ = std::make_shared<ParamListener>(get_node());
      params_ = param_listener_->get_params();
    }
    catch (const std::exception& e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(this->get_node()->get_logger(), "on_init() successful");

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration MobileBaseController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    if (dof_ == 0)
    {
      fprintf(stderr,
              "During ros2_control interface configuration, degrees of freedom is not valid;"
              " it should be positive. Actual DOF is %zu\n",
              dof_);
      std::exit(EXIT_FAILURE);
    }
    conf.names.reserve(dof_ * params_.command_interfaces.size());
    for (const auto& joint_name : params_.joints)
    {
      for (const auto& interface_type : params_.command_interfaces)
      {
        conf.names.push_back(joint_name + "/" + interface_type);
      }
    }

    return conf;
  }

  controller_interface::InterfaceConfiguration MobileBaseController::state_interface_configuration() const
  {
    return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::CallbackReturn MobileBaseController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
  {
    params_ = param_listener_->get_params();

    node_name_ = std::string(get_node()->get_name());

    dof_ = params_.joints.size();

    command_joint_names_ = params_.joints;

    command_interface_names_.clear();
    for (const auto& joint_name : params_.joints)
    {
      for (const auto& command_interface_type : params_.command_interfaces)
      {
        command_interface_names_.push_back(joint_name + "/" + command_interface_type);
      }
    }

    joints_cmd_sub_ = this->get_node()->create_subscription<DataType>(
      "~/commands",
      rclcpp::SystemDefaultsQoS(),
      [this](const DataType::SharedPtr msg)
      {
        // check if message is correct size, if not ignore
        if (msg->data.size() == command_interface_names_.size())
        {
          rt_buffer_ptr_.writeFromNonRT(msg);
        }
        else
        {
          RCLCPP_ERROR(this->get_node()->get_logger(),
                       "Invalid command received of %zu size, expected %zu size",
                       msg->data.size(),
                       command_interface_names_.size());
        }
      });

    // pre-reserve command interfaces
    command_interfaces_.reserve(command_interface_names_.size());

    RCLCPP_INFO(this->get_node()->get_logger(), "configure successful");

    // The names should be in the same order as for command interfaces for easier matching
    reference_interface_names_ = command_interface_names_;
    // for any case make reference interfaces size of command interfaces
    reference_interfaces_.resize(reference_interface_names_.size(), std::numeric_limits<double>::quiet_NaN());

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn MobileBaseController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
  {
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_interfaces;
    if (!controller_interface::get_ordered_interfaces(
          command_interfaces_, command_interface_names_, std::string(""), ordered_interfaces) ||
        command_interface_names_.size() != ordered_interfaces.size())
    {
      RCLCPP_ERROR(this->get_node()->get_logger(),
                   "Expected %zu command interfaces, got %zu",
                   command_interface_names_.size(),
                   ordered_interfaces.size());
      return controller_interface::CallbackReturn::ERROR;
    }

    // reset command buffer if a command came through callback when controller was inactive
    rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<DataType>>(nullptr);

    RCLCPP_INFO(this->get_node()->get_logger(), "activate successful");

    std::fill(reference_interfaces_.begin(), reference_interfaces_.end(), std::numeric_limits<double>::quiet_NaN());

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn MobileBaseController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // reset command buffer
    rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<DataType>>(nullptr);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  bool MobileBaseController::on_set_chained_mode(bool /*chained_mode*/)
  {
    return true;
  }

  controller_interface::return_type MobileBaseController::update_and_write_commands(const rclcpp::Time& /*time*/,
                                                                                    const rclcpp::Duration& /*period*/)
  {
    for (size_t i = 0; i < command_interfaces_.size(); ++i)
    {
      if (!std::isnan(reference_interfaces_[i]))
      {
        command_interfaces_[i].set_value(reference_interfaces_[i]);
      }
    }

    return controller_interface::return_type::OK;
  }

  std::vector<hardware_interface::CommandInterface> MobileBaseController::on_export_reference_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> reference_interfaces;

    for (size_t i = 0; i < reference_interface_names_.size(); ++i)
    {
      reference_interfaces.push_back(hardware_interface::CommandInterface(
        get_node()->get_name(), reference_interface_names_[i], &reference_interfaces_[i]));
    }

    return reference_interfaces;
  }

  controller_interface::return_type MobileBaseController::update_reference_from_subscribers(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
  {
    auto joint_commands = rt_buffer_ptr_.readFromRT();
    // message is valid
    if (!(!joint_commands || !(*joint_commands)))
    {
      if (reference_interfaces_.size() != (*joint_commands)->data.size())
      {
        RCLCPP_ERROR_THROTTLE(get_node()->get_logger(),
                              *(get_node()->get_clock()),
                              1000,
                              "command size (%zu) does not match number of reference interfaces (%zu)",
                              (*joint_commands)->data.size(),
                              reference_interfaces_.size());
        return controller_interface::return_type::ERROR;
      }
      reference_interfaces_ = (*joint_commands)->data;
    }

    return controller_interface::return_type::OK;
  }

}   // namespace mobile_base_controller

PLUGINLIB_EXPORT_CLASS(mobile_base_controller::MobileBaseController, controller_interface::ChainableControllerInterface)
