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

#ifndef ROBAST_NAV_POSES_IMPORTER__POSES_IMPORTER_HPP_
#define ROBAST_NAV_POSES_IMPORTER__POSES_IMPORTER_HPP_

#include <memory>
#include <string>

#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include "robast_msgs/srv/import_yaml_poses.hpp"
#include "yaml-cpp/yaml.h"

namespace robast_nav_poses_importer
{

/**
 * @class robast_nav_poses_importer::YamlPosesImporter
 * @brief A server that gets a path to a goal position and selects from a list of poses the one
 * that's closest to the path and returns this one as interim goal pose.
 */
class YamlPosesImporter : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for robast_nav_poses_importer::YamlPosesImporter class
   */
  YamlPosesImporter();
  /**
   * @brief A destructor for robast_nav_poses_importer::YamlPosesImporter class
   */
  ~YamlPosesImporter();

protected:
  /**
   * @brief Configures member variables
   *
   * Initializes action server for "YamlPosesImporter"
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

  /**
   * @brief Helper function to load the poses from a yaml file
   * @param yaml_filename full path to the yaml to load the poses from
   */
  void load_poses_from_yaml(const std::string yaml_filename);

  /**
   * @brief Callback function for the service that creates the service response provided with the poses
   * @param request_header Service request header
   * @param request Service request
   * @param response Service response
   */
  void providePosesCallback(
        const std::shared_ptr<rmw_request_id_t>/*request_header*/,
        const std::shared_ptr<robast_msgs::srv::ImportYamlPoses::Request> request,
        std::shared_ptr<robast_msgs::srv::ImportYamlPoses::Response> response);

  rclcpp::Service<robast_msgs::srv::ImportYamlPoses>::SharedPtr import_poses_service_;
  
  std::vector<geometry_msgs::msg::PoseStamped> poses_;
};

}  // namespace robast_nav_poses_importer

#endif  // ROBAST_NAV_POSES_IMPORTER__POSES_IMPORTER_HPP_
