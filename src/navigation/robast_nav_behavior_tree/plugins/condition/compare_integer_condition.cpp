// Copyright (c) 22021 Tobias Alscher
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <vector>
#include "robast_nav_behavior_tree/plugins/condition/compare_integer_condition.hpp"

namespace nav2_behavior_tree
{

CompareIntegerCondition::CompareIntegerCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
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

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::CompareIntegerCondition>("CompareIntegerA>BCondition");
}