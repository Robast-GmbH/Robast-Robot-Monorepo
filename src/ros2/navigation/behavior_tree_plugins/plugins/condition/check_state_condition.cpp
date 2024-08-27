#include "behavior_tree_plugins/plugins/condition/check_state_condition.hpp"

#include <string>
#include <vector>

namespace nav2_behavior_tree
{

  CheckStateCondition::CheckStateCondition(const std::string& condition_name, const BT::NodeConfiguration& conf)
      : BT::ConditionNode(condition_name, conf)
  {
    getInput("variable_name", _name);
  }

  BT::NodeStatus CheckStateCondition::tick()
  {
    std::string current_state;
    config().blackboard->get<std::string>(_name, current_state);

    if (current_state == "true")
    {
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::FAILURE;
    }
  }

}   // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::CheckStateCondition>("StateCheck");
}