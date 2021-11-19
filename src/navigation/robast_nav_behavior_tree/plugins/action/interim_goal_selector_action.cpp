#ifndef __INTERIM_GOAL_SELECTOR_ACTION_H__
#define __INTERIM_GOAL_SELECTOR_ACTION_H__

#include <memory>
#include <string>

#include "robast_msgs/action/compute_interim_goal.hpp"
#include "robast_nav_behavior_tree/plugins/action/interim_goal_selector_action.hpp"

namespace nav2_behavior_tree
{

InterimGoalCompAction::InterimGoalCompAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<robast_msgs::action::ComputeInterimGoal>(xml_tag_name, action_name, conf)
{
}

void InterimGoalCompAction::on_tick()
{
  getInput("path", goal_.path);
  getInput("pose", goal_.pose);
  getInput("controller_id", goal_.controller_id);
}

void InterimGoalCompAction::on_wait_for_result()
{
  // Grab the new path
  nav_msgs::msg::Path new_path;
  getInput("path", new_path);

  // Check if it is not same with the current one
  if (goal_.path != new_path) {
    // the action server on the next loop iteration
    goal_.path = new_path;
    goal_updated_ = true;
  }  

  std::string new_controller_id;
  getInput("controller_id", new_controller_id);

  if (goal_.controller_id != new_controller_id) 
  {
    goal_.controller_id = new_controller_id;
    goal_updated_ = true;
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::InterimGoalCompAction>(
        name, "interim_goal_selector", config);
    };

  factory.registerBuilder<nav2_behavior_tree::InterimGoalCompAction>(
    "InterimGoalCompAction", builder);
}


#endif // __INTERIM_GOAL_SELECTOR_ACTION_H__