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
#include "behavior_tree_plugins/plugins/condition/is_incremented_condition.hpp"

namespace nav2_behavior_tree
{

IsIncrementedCondition::IsIncrementedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{  
  getInput("variable_name", _name);
}

BT::NodeStatus IsIncrementedCondition::tick()
{
  std::string current_state = "0";
  config().blackboard->get<std::string>(_name, current_state);

  int current_state_int = std::stoi(current_state);

  if (current_state_int > _state) {
    std::cout << "IsIncrementedCondition: Value of input variable has been incremented! _state is " << std::to_string(_state) << " and current_state_int is " << std::to_string(current_state_int) << std::endl;
    _state = current_state_int;
    return BT::NodeStatus::SUCCESS;
  }

  _state = current_state_int;
  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsIncrementedCondition>("IsIncremented");
}