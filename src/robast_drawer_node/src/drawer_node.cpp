#include <inttypes.h>
#include <memory>

#include "robast_drawer_node/drawer_node.hpp"

namespace robast_drawer_node
{


DrawerController::DrawerController()
: nav2_util::LifecycleNode("robast_drawer_node", "", true)
{
  RCLCPP_INFO(get_logger(), "Creating");
}

DrawerController::~DrawerController()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_util::CallbackReturn DrawerController::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  
  action_server_ = std::make_unique<ActionServer>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "drawer_controller", std::bind(&DrawerController::open_drawer, this), false);

  RCLCPP_INFO(get_logger(), "End of Configuring");

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn DrawerController::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  action_server_->activate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn DrawerController::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn DrawerController::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  action_server_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn DrawerController::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void DrawerController::open_drawer()
{
  auto goal = action_server_->get_current_goal();
  auto feedback = std::make_shared<ActionT::Feedback>();
  auto result = std::make_shared<ActionT::Result>();

  if (!is_request_valid(goal, result)) {
    return;
  }

  // Get the interim goals from the action goal
  geometry_msgs::msg::PoseStamped final_pose = goal->path.poses[goal->path.poses.size() - 1];
  // Clean the interim_goals_ vector, which is quite important as there could be entries stored in the vector from the last run
  interim_goals_.clear();
  for (u_int16_t i = 0; i < goal->poses.size(); i++)
  {
    interim_goal interim_goal;
    interim_goal.pose = goal->poses[i];
    interim_goal.dist_to_final_pose = calculate_euclidean_distance(final_pose.pose.position.x,
                                                                  final_pose.pose.position.y,
                                                                  interim_goal.pose.pose.position.x,
                                                                  interim_goal.pose.pose.position.y);
    // Only add those interim goals to the list of possible interim_goals_ that are within the search radius
    if (interim_goal.dist_to_final_pose < goal->search_radius)
    {
      interim_goals_.push_back(interim_goal);
    }
  }

  // From all possible interim goals find the k nearest neighbors to the final goal pose
  filter_k_nearest_neighbors_interim_goals();

  bool result_state = select_final_interim_goal_on_path(goal->path);
  if (result_state == false)
  {
    RCLCPP_ERROR(get_logger(), "No interim goal on path!");
    action_server_->terminate_current(result);
  }
  else
  {
    send_succeeded_action_result(goal, result);
  }
}


bool DrawerController::is_request_valid(
  const std::shared_ptr<const typename ActionT::Goal> goal,
  std::shared_ptr<ActionT::Result> result)
{
  if (!action_server_ || !action_server_->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server inactive. Stopping.");
    return false;
  } 

  if (goal->path.poses.empty()) {
    RCLCPP_ERROR(get_logger(), "Invalid path, Path is empty.");
    action_server_->terminate_current(result);
    return false;
  }

  if (goal->search_radius <= 0.0) {
    RCLCPP_ERROR(get_logger(), "Invalid search radius: %f! Search radius has to be > 0.0!", goal->search_radius);
    action_server_->terminate_current(result);
    return false;
  }

  RCLCPP_INFO(
    get_logger(), "Received a interim goal computation request for a path with %i poses, %i possible interim goals and search radius %f.",
    static_cast<int>(goal->path.poses.size()), goal->poses.size(), goal->search_radius);
  return true;
}


void DrawerController::send_succeeded_action_result(
  const std::shared_ptr<const typename ActionT::Goal> goal,
  std::shared_ptr<ActionT::Result> result)
{
  if (goal->is_path_reversed) {
      result->waypoint_index = goal->path.poses.size() - waypoint_index_;
      RCLCPP_INFO(get_logger(), "Found an interim pose on reversed path to goal with x-coordinate %f and y-coordinate %f. The waypoint that was first inside the max_interim_dist_to_path was the one with the index %i!",
      interim_goals_[0].pose.pose.position.x, interim_goals_[0].pose.pose.position.y, result->waypoint_index);
    } else {
      result->waypoint_index = waypoint_index_;
      RCLCPP_INFO(get_logger(), "Found an interim pose on path to goal with x-coordinate %f and y-coordinate %f. The waypoint that was first inside the max_interim_dist_to_path was the one with the index %i!",
      interim_goals_[0].pose.pose.position.x, interim_goals_[0].pose.pose.position.y, result->waypoint_index);
    }

    result->interim_pose = interim_goals_[0].pose;
    action_server_->succeeded_current(result);
}

} // namespace robast_drawer_node