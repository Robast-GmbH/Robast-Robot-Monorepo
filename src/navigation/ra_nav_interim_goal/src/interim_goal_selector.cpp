#include "ra_nav_interim_goal/interim_goal_selector.hpp"

#include "yaml-cpp/yaml.h"
#include "rclcpp/rclcpp.hpp"
#include "ra_nav_interim_goal/action/compute_interim_goal.hpp"
#include <inttypes.h>
#include <memory>

#define param_interim_goals_yaml "interim_goals"

namespace ra_nav_interim_goal
{

InterimGoalSelector::InterimGoalSelector()
: nav2_util::LifecycleNode("ra_nav_interim_goal", "", true),
{
  RCLCPP_INFO(get_logger(), "Creating");
  
  // Declare this node's parameters
  declare_parameter(param_interim_goals_yaml);
}

InterimGoalSelector::~InterimGoalSelector()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_util::CallbackReturn
InterimGoalSelector::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  std::string interim_goals_yaml_filename = get_parameter(param_interim_goals_yaml).as_string();

  load_interim_goals_from_yaml(interim_goals_yaml_filename);

  action_server_ = std::make_unique<ActionServer>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "ComputeInterimGoal", std::bind(&InterimGoalSelector::select_interim_goal, this), false);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
InterimGoalSelector::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  action_server_->activate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
InterimGoalSelector::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
InterimGoalSelector::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  action_server_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
InterimGoalSelector::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void InterimGoalSelector::select_interim_goal() {
  auto goal = action_server_->get_current_goal();
  auto feedback = std::make_shared<ActionT::Feedback>();
  auto result = std::make_shared<ActionT::Result>();

  // Check if request is valid
  if (!action_server_ || !action_server_->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server inactive. Stopping.");
    return;
  }

  RCLCPP_INFO(
    get_logger(), "Received a interim goal computation request for a path with %i poses.",
    static_cast<int>(goal->goal_path.poses.size()));

}

void InterimGoalSelector::load_interim_goals_from_yaml(const std::string interim_goals_yaml_filename)
{
  //TODO: get full path for interim_goals_yaml_filename

  YAML::Node doc = YAML::LoadFile(interim_goals_yaml_filename);

  for (std::size_t i=1; i<doc.size()+1; i++) {
    interim_goal interim_goal;
    interim_goal.x = doc[i]["x"].as<double>();
    interim_goal.y = doc[i]["y"].as<double>();
    interim_goal.yaw = doc[i]["yaw"].as<double>();

    interim_goals_.push_back(interim_goal);
  }
}

} // namespace ra_nav_door_bell