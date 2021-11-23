#include <inttypes.h>
#include <memory>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "robast_nav_interim_goal/interim_goal_selector.hpp"


#define param_interim_goals_yaml "interim_goals_yaml"
#define param_k_nearest_neighbors "k_nearest_neighbors" 
#define param_max_interim_dist_to_path "max_interim_dist_to_path" 

namespace robast_nav_interim_goal
{


InterimGoalSelector::InterimGoalSelector()
: nav2_util::LifecycleNode("robast_nav_interim_goal", "", true)
{
  RCLCPP_INFO(get_logger(), "Creating");
  
  // Declare this node's parameters
  declare_parameter(param_interim_goals_yaml);
  declare_parameter(param_k_nearest_neighbors);
  declare_parameter(param_max_interim_dist_to_path);
}

InterimGoalSelector::~InterimGoalSelector()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_util::CallbackReturn InterimGoalSelector::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  k_nearest_neighbors_ = get_parameter(param_k_nearest_neighbors).as_int();
  epsilon_ = get_parameter(param_max_interim_dist_to_path).as_double();
  std::string interim_goals_yaml_filename = get_parameter(param_interim_goals_yaml).as_string();

  //load_interim_goals_from_yaml(interim_goals_yaml_filename);
  
  action_server_ = std::make_unique<ActionServer>(
    rclcpp_node_,
    "ComputeInterimGoal", std::bind(&InterimGoalSelector::select_interim_goal, this), false);

  RCLCPP_INFO(get_logger(), "End of Configuring");

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn InterimGoalSelector::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  action_server_->activate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn InterimGoalSelector::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn InterimGoalSelector::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  action_server_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn InterimGoalSelector::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void InterimGoalSelector::select_interim_goal()
{
    RCLCPP_INFO(get_logger(), "select mamamia");

  auto goal = action_server_->get_current_goal();
  auto feedback = std::make_shared<ActionT::Feedback>();
  auto result = std::make_shared<ActionT::Result>();

  if (!is_request_valid(goal, result)) {
    return;
  }

  // From all possible interim goals find the k nearest neighbors to the final goal pose
  filter_k_nearest_neighbors_interim_goals(goal->pose);

  bool result_state = select_final_interim_goal_on_path(goal->path);
  if (result_state == false)
  {
    RCLCPP_ERROR(get_logger(), "No interim goal on path!");
    action_server_->terminate_current(result);
  }
  else
  {
    RCLCPP_INFO(
    get_logger(), "Found an interim pose on path to goal with x-coordinate %d and y-coordinate %d.",
    interim_goals_[0].x, interim_goals_[0].y);

    result->interim_pose.pose.position.x = interim_goals_[0].x;
    result->interim_pose.pose.position.y = interim_goals_[0].y;

    //transform euler pose orientation to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, interim_goals_[0].yaw);

    result->interim_pose.pose.orientation = tf2::toMsg(q);
    action_server_->succeeded_current(result);
  }
}

void InterimGoalSelector::filter_k_nearest_neighbors_interim_goals(geometry_msgs::msg::PoseStamped final_pose)
{
  for (u_int16_t i = 0; i < interim_goals_.size(); i++)
  {
    interim_goal interim_goal = interim_goals_[i];
    interim_goals_[i].dist_to_final_pose = calculate_euclidean_distance(final_pose.pose.position.x,
                                                            final_pose.pose.position.y,
                                                            interim_goal.x,
                                                            interim_goal.y);
  }
  // sort the vector of interim_goals_, so that interim_goals_[0] contains the smallest distance to the final pose
  std::sort(interim_goals_.begin(), interim_goals_.end(), compare_dist_to_final_pose);

  interim_goals_.resize(k_nearest_neighbors_);
}

bool InterimGoalSelector::select_final_interim_goal_on_path(nav_msgs::msg::Path path)
{
  for(int i = (sizeof(path.poses) / sizeof(path.poses[0])); i>=0; --i)
  {
    double path_x = path.poses[i].pose.position.x;
    double path_y = path.poses[i].pose.position.y;
    for(long unsigned int j = 0; j < interim_goals_.size(); j++)
    {
      if(calculate_euclidean_distance(path_x, path_y, interim_goals_[j].x, interim_goals_[j].y) < epsilon_)
      {
        auto final_interim_goal = interim_goals_[j];
        interim_goals_.resize(1);
        interim_goals_[0] = final_interim_goal;
        return true;
      }
    }
  }
  return false;
}

double InterimGoalSelector::calculate_euclidean_distance(double x1, double y1, double x2, double y2)
{
  return std::hypot(x1 - x2, y1 - y2);
}

bool InterimGoalSelector::is_request_valid(
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

  RCLCPP_INFO(
    get_logger(), "Received a interim goal computation request for a path with %i poses.",
    static_cast<int>(goal->path.poses.size()));
  return true;
}

void InterimGoalSelector::load_interim_goals_from_yaml(const std::string interim_goals_yaml_filename)
{
  YAML::Node doc = YAML::LoadFile(interim_goals_yaml_filename);

  for (std::size_t i=1; i<doc.size()+1; i++) {
    interim_goal interim_goal;
    interim_goal.x = doc[i]["x"].as<double>();
    interim_goal.y = doc[i]["y"].as<double>();
    interim_goal.yaw = doc[i]["yaw"].as<double>();
    interim_goal.dist_to_final_pose = 0;

    interim_goals_.push_back(interim_goal);
  }
}

} // namespace robast_nav_door_bell