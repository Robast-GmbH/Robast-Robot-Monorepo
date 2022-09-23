// Copyright (c) 2019 Samsung Research America
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

#ifndef ROBAST_NAV_RECOVERIES__RECOVERIES_COSTMAP_HPP_
#define ROBAST_NAV_RECOVERIES__RECOVERIES_COSTMAP_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <functional>


#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace robast_nav_recoveries
{

/**
 * @class robast_nav_recoveries::RecoveriesCostmap
 * @brief A server that gets a path to a goal position and selects from a list of poses the one
 * that's closest to the path and returns this one as interim goal pose.
 */
class RecoveriesCostmap : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for robast_nav_recoveries::RecoveriesCostmap class
   */
  explicit RecoveriesCostmap(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief A destructor for robast_nav_recoveries::RecoveriesCostmap class
   */
  ~RecoveriesCostmap();

protected:
  /**
   * @brief Configures member variables
   *
   * Initializes action server for "RecoveriesCostmap"
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Resets member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
  std::string costmap_namespace_;
};
}  // namespace robast_nav_recoveries

#endif  // ROBAST_NAV_RECOVERIES__RECOVERIES_COSTMAP_HPP_
