// Copyright (c) 2019 Intel Corporation
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

#include <chrono>
#include <vector>
#include <memory>
#include <string>
#include <utility>
#include <limits>

#include "robast_nav_recoveries/recoveries_costmap.hpp"


namespace robast_nav_recoveries
{

RecoveriesCostmap::RecoveriesCostmap(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("recoveries_costmap", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating recoveries costmap");

  declare_parameter("costmap_namespace", rclcpp::ParameterValue(std::string("recoveries_costmap")));

  get_parameter("costmap_namespace", costmap_namespace_);

  RCLCPP_INFO(get_logger(), "costmap_namespace: %s", costmap_namespace_.c_str());

  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    costmap_namespace_, std::string{get_namespace()}, costmap_namespace_);

  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);  
}

RecoveriesCostmap::~RecoveriesCostmap()
{
  RCLCPP_INFO(get_logger(), "Destroying recoveries costmap");
  costmap_thread_.reset();
}

nav2_util::CallbackReturn
RecoveriesCostmap::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring recoveries costmap");

  costmap_ros_->on_configure(state);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RecoveriesCostmap::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating recoveries costmap");

  costmap_ros_->on_activate(state);

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RecoveriesCostmap::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating recoveries costmap");

  costmap_ros_->on_deactivate(state);

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RecoveriesCostmap::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up recoveries costmap");

  costmap_ros_->on_cleanup(state);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RecoveriesCostmap::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down recoveries costmap");
  return nav2_util::CallbackReturn::SUCCESS;
}

}  // namespace nav2_controller

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robast_nav_recoveries::RecoveriesCostmap)