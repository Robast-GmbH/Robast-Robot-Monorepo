// Copyright (c) 2018 Intel Corporation
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

#include "ros_bt_basics/bt_base_node.hpp"

#include <memory>
#include <string>
#include <utility>
#include <set>
#include <limits>
#include <vector>

namespace ros_bt_basics
{

  BtNavigator::BtNavigator(const rclcpp::NodeOptions& options)
    : nav2_util::LifecycleNode("bt_navigator", "", options)
  {
    RCLCPP_INFO(get_logger(), "Creating");

    const std::vector<std::string> plugin_libs = {
    "change_led_action_bt_node",
    "open_drawer_action_bt_node",
    "drawer_open_request_action_bt_node",
    "drawer_status_condition_bt_node",
    "example_action_bt_node"
    };

    declare_parameter("plugin_lib_names", plugin_libs);

  }

  BtNavigator::~BtNavigator()
  {}

  nav2_util::CallbackReturn
    BtNavigator::on_configure(const rclcpp_lifecycle::State& /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Configuring");

    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface());
    

    // Libraries to pull plugins (BT Nodes) from
    auto plugin_lib_names = get_parameter("plugin_lib_names").as_string_array();



    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
    BtNavigator::on_activate(const rclcpp_lifecycle::State& /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Activating");


    // create bond connection
    createBond();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
    BtNavigator::on_deactivate(const rclcpp_lifecycle::State& /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Deactivating");

    // destroy bond connection
    destroyBond();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
    BtNavigator::on_cleanup(const rclcpp_lifecycle::State& /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up");


    RCLCPP_INFO(get_logger(), "Completed Cleaning up");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
    BtNavigator::on_shutdown(const rclcpp_lifecycle::State& /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Shutting down");
    return nav2_util::CallbackReturn::SUCCESS;
  }

} 