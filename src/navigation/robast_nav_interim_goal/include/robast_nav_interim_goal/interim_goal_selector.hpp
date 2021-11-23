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
#include <tf2/LinearMath/Quaternion.h>

#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robast_msgs/action/compute_interim_goal.hpp"
#include "yaml-cpp/yaml.h"

namespace robast_nav_interim_goal
{

struct interim_goal
{
  double x;
  double y;
  double yaw;
  double dist_to_final_pose;
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
   * @brief Helper function to check if the request for the action is valid
   * @param goal Action goal that is checked for validity
   * @param result Action result that is used in case the request is invalid and action is canceled
   * @return True if valid, false if not valid
   */
  bool is_request_valid(const std::shared_ptr<const typename ActionT::Goal> goal,
                        std::shared_ptr<ActionT::Result> result);

  /**
   * @brief Helper function to find the k nearest neighbors closest to the final goal pose
   * @param k Number of closest neighbors to find
   * @param final_pose Pose to which the neighbors should be found
   */
  void filter_k_nearest_neighbors_interim_goals(geometry_msgs::msg::PoseStamped final_pose);

  /**
   * @brief selects the final interim goal on the path with given epsilon
   * @param path path to the goal
   * @return true if a goal was selected. False if none is on the path
   */
  bool select_final_interim_goal_on_path(nav_msgs::msg::Path path);

  /**
   * @brief Helper function to calculate the euclidean distance between two 2D-points
   * @param x1 x-coordinate of the first point
   * @param y1 y-coordinate of the first point
   * @param x2 x-coordinate of the second point
   * @param y2 y-coordinate of the second point
   * @return Euclidean distance between the two 2D-points
   */
  double calculate_euclidean_distance(double x1, double y1, double x2, double y2);


  /**
   * @brief Action server execution callback
   */
  void select_interim_goal();

  // Our action server
  std::unique_ptr<ActionServer> action_server_;

  int64_t k_nearest_neighbors_;
  std::vector<interim_goal> interim_goals_;
  double epsilon_;
};


  /**
   * @brief Helper function to compare the distance to the final pose of two interim goals
   * @param interim_goal1 
   * @param interim_goal2
   * @return True if distance to final pose of interim_goal1 is smaller than the one of interim_goal2
   */
  static bool compare_dist_to_final_pose(interim_goal interim_goal1, interim_goal interim_goal2)
  {
    return (interim_goal1.dist_to_final_pose < interim_goal2.dist_to_final_pose);
  }
}  // namespace robast_nav_interim_goal

#endif  // ROBAST_NAV_INTERIM_GOAL__INTERIM_GOAL_HPP_
