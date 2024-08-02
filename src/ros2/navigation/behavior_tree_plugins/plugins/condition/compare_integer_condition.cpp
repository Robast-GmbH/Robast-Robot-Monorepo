#include "behavior_tree_plugins/plugins/condition/compare_integer_condition.hpp"

#include <string>
#include <vector>

namespace nav2_behavior_tree
{

  CompareIntegerCondition::CompareIntegerCondition(const std::string& condition_name, const BT::NodeConfiguration& conf)
      : BT::ConditionNode(condition_name, conf)
  {
    getInput("first_integer", _first_integer_name);
    getInput("second_integer", _second_integer_name);
  }

  BT::NodeStatus CompareIntegerCondition::tick()
  {
    int first_integer = 0;
    int second_integer = 0;
    config().blackboard->get<int>(_first_integer_name, first_integer);
    config().blackboard->get<int>(_second_integer_name, second_integer);

    if (first_integer > second_integer)
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
  factory.registerNodeType<nav2_behavior_tree::CompareIntegerCondition>("CompareIntegerA>BCondition");
}