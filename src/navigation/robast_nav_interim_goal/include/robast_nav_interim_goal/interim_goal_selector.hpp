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

#ifndef ROBAST_NAV_INTERIM_GOAL__INTERIM_GOAL_HPP_
#define ROBAST_NAV_INTERIM_GOAL__INTERIM_GOAL_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "nav2_util/lifecycle_node.hpp"
#include "robast_msgs/action/compute_interim_goal.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace robast_nav_interim_goal
{

struct interim_goal
{
  double x;
  double y;
  double yaw;
};

enum class ActionStatus
{
  UNKNOWN = 0,
  PROCESSING = 1,
  FAILED = 2,
  SUCCEEDED = 3
};

/**
 * @class robast_nav_interim_goal::InterimGoalSelector
 * @brief A server that gets a path to a goal position and selects from a list of poses the one
 * that's closest to the path and returns this one as interim goal pose.
 */
class InterimGoalSelector : public nav2_util::LifecycleNode
{
public:
  using ActionT = robast_msgs::action::ComputeInterimGoal;
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;

  /**
   * @brief A constructor for robast_nav_interim_goal::InterimGoalSelector class
   */
  InterimGoalSelector();
  /**
   * @brief A destructor for robast_nav_interim_goal::InterimGoalSelector class
   */
  ~InterimGoalSelector();

protected:
  /**
   * @brief Configures member variables
   *
   * Initializes action server for "FollowWaypoints"
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
   * @brief Helper function to load the interim goals from a yaml file
   * @param state Reference to LifeCycle node state
   */
  void load_interim_goals_from_yaml(const std::string interim_goals_yaml_filename);

  /**
   * @brief Action server execution callback
   */
  void select_interim_goal();

  // TODO: Remove these variables. Only here for comparison
  // Our action server
  std::unique_ptr<ActionServer> action_server_;
  bool stop_on_failure_;
  ActionStatus current_goal_status_;
  int loop_rate_;
  std::vector<int> failed_ids_;

  // TODO: Remove this comment
  // NEUE VON MIR HINZUGEFÜGTE:
  std::vector<interim_goal> interim_goals_;
};

}  // namespace robast_nav_interim_goal

#endif  // ROBAST_NAV_INTERIM_GOAL__INTERIM_GOAL_HPP_
