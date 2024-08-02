#include "behavior_tree_plugins/plugins/condition/variable_condition_updated.hpp"

#include <string>

namespace nav2_behavior_tree
{

  VariableUpdatedCondition::VariableUpdatedCondition(const std::string& condition_name,
                                                     const BT::NodeConfiguration& conf)
      : BT::ConditionNode(condition_name, conf)
  {
    getInput("variable_name", _name);
  }

  BT::NodeStatus VariableUpdatedCondition::tick()
  {
    if (status() == BT::NodeStatus::IDLE)
    {
      config().blackboard->get<std::string>(_name, _state);
      return BT::NodeStatus::FAILURE;
    }

    std::string current_state;
    config().blackboard->get<std::string>(_name, current_state);

    if (current_state != _state)
    {
      _state = current_state;
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

}   // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::VariableUpdatedCondition>("VariableUpdated");
}