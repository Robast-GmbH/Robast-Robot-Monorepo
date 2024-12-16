#include "bt_plugins/condition/double_compare_condition.hpp"

#include <stdio.h>

namespace statemachine
{
  DoubleCompareCondition::DoubleCompareCondition(const std::string &name, const BT::NodeConfig &config)
      : BT::ConditionNode(name, config)
  {
  }

  BT::NodeStatus DoubleCompareCondition::tick()
  {
    // Retrieve input values from the ports
    getInput("target_value", _target_value);
    getInput("value", _value);
    getInput("comparison", _comparison);
    // Perform the comparison based on the specified comparison type
    if (_comparison == "==")
    {
      return (_value == _target_value) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    else if (_comparison == "!=")
    {
      return (_value != _target_value) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    else if (_comparison == "<")
    {
      return (_value < _target_value) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    else if (_comparison == "<=")
    {
      return (_value <= _target_value) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    else if (_comparison == ">")
    {
      return (_value > _target_value) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    else if (_comparison == ">=")
    {
      return (_value >= _target_value) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    else
    {
      return BT::NodeStatus::FAILURE;   // Invalid comparison type
    }
  }
}   // namespace statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<statemachine::DoubleCompareCondition>("DoubleCompareCondition");
}