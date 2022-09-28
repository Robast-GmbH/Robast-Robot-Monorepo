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
#include "behavior_tree_plugins/plugins/condition/variable_condition_updated.hpp"

namespace nav2_behavior_tree
{

VariableUpdatedCondition::VariableUpdatedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{  
  getInput("variable_name", _name);
}

BT::NodeStatus VariableUpdatedCondition::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    config().blackboard->get<std::string>(_name, _state);
    return BT::NodeStatus::FAILURE;
  }

  std::string current_state;
  config().blackboard->get<std::string>(_name, current_state);

  if (current_state != _state) {
    _state = current_state;
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::VariableUpdatedCondition>("VariableUpdated");
}