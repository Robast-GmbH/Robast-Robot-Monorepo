#ifndef ROS2_CONTROL_PLUGIN_DRYVE_D1_HPP_
#define ROS2_CONTROL_PLUGIN_DRYVE_D1_HPP_

#include <chrono>
#include <cmath>
#include <limits>
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
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface_utils/hardware_interface_utils.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2_control_plugin_dryve_d1/visibility_control.h"

namespace ros2_control_plugin_dryve_d1
{
  class DryveD1SystemHardware : public hardware_interface::SystemInterface
  {
   public:
    RCLCPP_SHARED_PTR_DEFINITIONS(DryveD1SystemHardware)

    ROS2_CONTROL_PLUGIN_DRYVE_D1_PUBLIC hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo& info) override;

    ROS2_CONTROL_PLUGIN_DRYVE_D1_PUBLIC hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_PLUGIN_DRYVE_D1_PUBLIC std::vector<hardware_interface::StateInterface> export_state_interfaces()
        override;

    ROS2_CONTROL_PLUGIN_DRYVE_D1_PUBLIC std::vector<hardware_interface::CommandInterface> export_command_interfaces()
        override;

    ROS2_CONTROL_PLUGIN_DRYVE_D1_PUBLIC hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_PLUGIN_DRYVE_D1_PUBLIC hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_PLUGIN_DRYVE_D1_PUBLIC hardware_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_PLUGIN_DRYVE_D1_PUBLIC hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_PLUGIN_DRYVE_D1_PUBLIC hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_PLUGIN_DRYVE_D1_PUBLIC hardware_interface::return_type read(const rclcpp::Time& time,
                                                                             const rclcpp::Duration& period) override;

    ROS2_CONTROL_PLUGIN_DRYVE_D1_PUBLIC hardware_interface::return_type write(const rclcpp::Time& time,
                                                                              const rclcpp::Duration& period) override;

   private:
    std::vector<double> _hw_position_commands;
    std::vector<double> _hw_position_states;

    std::vector<double> _hw_velocity_commands;
    std::vector<double> _hw_velocity_states;

    std::unique_ptr<dryve_d1_bridge::D1> _dryve_d1;

    std::string _ip_address;

    std::string _logger = "DryveD1SystemHardware";

    bool _zero_velocity_published_last;

    // this is the ports we configured in the router that is forwarded to port 502 (default modbus port) of the dryve d1
    int _port = 3502;

    // can be either 1 or -1.There are probably better ways to handle this, but this does the job for now
    int _direction = 1;

    double _si_unit_factor;

    bool _is_prismatic_joint;

    bool is_dryve_error_present();

    void run_dryve_state_machine_after_error(std::string_view dryve_error);
  };

}   // namespace ros2_control_plugin_dryve_d1

#endif   // ROS2_CONTROL_PLUGIN_DRYVE_D1_HPP_
