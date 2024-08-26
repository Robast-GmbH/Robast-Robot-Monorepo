#include "behavior_tree_plugins/plugins/condition/is_incremented_condition.hpp"

#include <string>

namespace nav2_behavior_tree
{

  IsIncrementedCondition::IsIncrementedCondition(const std::string& condition_name, const BT::NodeConfiguration& conf)
      : BT::ConditionNode(condition_name, conf)
  {
    getInput("variable_name", _name);
  }

  BT::NodeStatus IsIncrementedCondition::tick()
  {
    std::string current_state = "0";
    config().blackboard->get<std::string>(_name, current_state);

    int current_state_int = std::stoi(current_state);

    if (current_state_int > _state)
    {
      std::cout << "IsIncrementedCondition: Value of input variable has been incremented! _state is "
                << std::to_string(_state) << " and current_state_int is " << std::to_string(current_state_int)
                << std::endl;
      _state = current_state_int;
      return BT::NodeStatus::SUCCESS;
    }

    _state = current_state_int;
    return BT::NodeStatus::FAILURE;
  }

}   // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsIncrementedCondition>("IsIncremented");
}