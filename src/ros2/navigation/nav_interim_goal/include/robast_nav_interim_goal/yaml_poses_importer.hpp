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
#include <inttypes.h>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "communication_interfaces/action/import_yaml_poses.hpp"
#include "yaml-cpp/yaml.h"

namespace robast_nav_poses_importer
{

/**
 * @class robast_nav_poses_importer::YamlPosesImporter
 * @brief A server that imports poses from a yaml
 */
class YamlPosesImporter : public nav2_util::LifecycleNode
{
public:
  using ActionT = communication_interfaces::action::ImportYamlPoses;
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;
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
   * @brief Helper function to check if the request for the action is valid
   * @param goal Action goal that is checked for validity
   * @param result Action result that is used in case the request is invalid and action is canceled
   * @return True if valid, false if not valid
   */
  bool is_request_valid(const std::shared_ptr<const typename ActionT::Goal> goal,
                        std::shared_ptr<ActionT::Result> result);

  /**
   * @brief Callback function for the service that creates the service response provided with the poses
   */
  void provide_poses();

  // Our action server
  std::unique_ptr<ActionServer> action_server_;

  std::map<std::string, std::vector<geometry_msgs::msg::PoseStamped>> yaml_filename_by_poses_;
};

}  // namespace robast_nav_poses_importer

#endif  // ROBAST_NAV_POSES_IMPORTER__POSES_IMPORTER_HPP_
