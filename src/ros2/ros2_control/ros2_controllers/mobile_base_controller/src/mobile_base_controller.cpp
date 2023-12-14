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
    if (_dof == 0)
    {
      fprintf(stderr,
              "During ros2_control interface configuration, degrees of freedom is not valid;"
              " it should be positive. Actual DOF is %zu\n",
              _dof);
      std::exit(EXIT_FAILURE);
    }
    conf.names.reserve(_dof * params_.command_interfaces.size());
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

    _use_stamped_vel = params_.use_stamped_vel;

    _dof = params_.joints.size();

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

    if (_use_stamped_vel)
    {
      _publisher_cmd_vel =
          this->get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(params_.cmd_vel_topic, 10);
    }
    else
    {
      _publisher_cmd_vel = this->get_node()->create_publisher<geometry_msgs::msg::Twist>(params_.cmd_vel_topic, 10);
    }

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

  std::variant<geometry_msgs::msg::Twist, geometry_msgs::msg::TwistStamped> MobileBaseController::compute_cmd_vel(
      const std::vector<double>& hw_velocity_commands)
  {
    // Please mind:
    // I found out empirically, that i have to change the sign of the computed cmd_vel to get the correct direction
    if (_use_stamped_vel)
    {
      geometry_msgs::msg::TwistStamped cmd_vel_stamped;
      cmd_vel_stamped.twist.linear.x = -hw_velocity_commands[0];
      cmd_vel_stamped.twist.linear.y = 0.0;
      cmd_vel_stamped.twist.linear.z = 0.0;
      cmd_vel_stamped.twist.angular.x = 0.0;
      cmd_vel_stamped.twist.angular.y = 0.0;
      cmd_vel_stamped.twist.angular.z = 0.0;
      // Set the stamp
      cmd_vel_stamped.header.stamp = this->get_node()->get_clock()->now();
      return cmd_vel_stamped;
    }
    else
    {
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = -hw_velocity_commands[0];
      cmd_vel.linear.y = 0.0;
      cmd_vel.linear.z = 0.0;
      cmd_vel.angular.x = 0.0;
      cmd_vel.angular.y = 0.0;
      cmd_vel.angular.z = 0.0;
      return cmd_vel;
    }
  }

  controller_interface::return_type MobileBaseController::update_and_write_commands(const rclcpp::Time& /*time*/,
                                                                                    const rclcpp::Duration& /*period*/)
  {
    for (size_t i = 0; i < command_interfaces_.size(); ++i)
    {
      if (!std::isnan(reference_interfaces_[i]))
      {
        auto cmd_vel = compute_cmd_vel(reference_interfaces_);

        if (std::holds_alternative<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr>(_publisher_cmd_vel))
        {
          auto publisher = std::get<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr>(_publisher_cmd_vel);
          if (std::holds_alternative<geometry_msgs::msg::Twist>(cmd_vel))
          {
            publisher->publish(std::get<geometry_msgs::msg::Twist>(cmd_vel));
          }
          else
          {
            RCLCPP_ERROR(rclcpp::get_logger("MobileBaseController"), "Expected Twist but got TwistStamped");
          }
        }
        else
        {
          auto publisher = std::get<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr>(_publisher_cmd_vel);
          if (std::holds_alternative<geometry_msgs::msg::TwistStamped>(cmd_vel))
          {
            publisher->publish(std::get<geometry_msgs::msg::TwistStamped>(cmd_vel));
          }
          else
          {
            RCLCPP_ERROR(rclcpp::get_logger("MobileBaseController"), "Expected TwistStamped but got Twist");
          }
        }
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
