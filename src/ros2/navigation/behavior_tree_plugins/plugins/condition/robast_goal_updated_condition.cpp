#include "behavior_tree_plugins/plugins/condition/robast_goal_updated_condition.hpp"

#include <string>
#include <vector>

namespace nav2_behavior_tree
{

  RobastGoalUpdatedCondition::RobastGoalUpdatedCondition(const std::string& condition_name,
                                                         const BT::NodeConfiguration& conf)
      : BT::ConditionNode(condition_name, conf)
  {
  }

  BT::NodeStatus RobastGoalUpdatedCondition::tick()
  {
    if (is_goal_updated_ == true)
    {
      config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", goal_);
      is_goal_updated_ = false;
      std::cout << "RobastGoalUpdatedCondition: Setting private goal_ variable!" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseStamped current_goal;
    config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", current_goal);

    if (goal_ != current_goal)
    {
      goal_ = current_goal;
      is_goal_updated_ = true;
      std::cout << "RobastGoalUpdatedCondition: goal_ != current_goal" << std::endl;
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

}   // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RobastGoalUpdatedCondition>("RobastGoalUpdated");
}