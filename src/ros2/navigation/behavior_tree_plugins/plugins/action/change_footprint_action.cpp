// Copyright (c) 2023 Jacob Ritterbach
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

#include "behavior_tree_plugins/plugins/action/change_footprint_action.hpp"

namespace nav2_behavior_tree
{

  ChangeFootprintAction::ChangeFootprintAction(const std::string& xml_tag_name,
                                               const std::string& action_name,
                                               const BT::NodeConfiguration& conf)
      : BtActionNode<communication_interfaces::action::ChangeFootprint>(xml_tag_name, action_name, conf)
  {
    getInput("footprint", goal_.footprint);
    getInput("time_until_reset_in_ms", goal_.time_until_reset_in_ms);
  }

  void ChangeFootprintAction::on_tick()
  {
  }

  BT::NodeStatus ChangeFootprintAction::on_success()
  {
    return BT::NodeStatus::SUCCESS;
  }

}   // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config)
  {
    return std::make_unique<nav2_behavior_tree::ChangeFootprintAction>(name, "change_footprint", config);
  };

  factory.registerBuilder<nav2_behavior_tree::ChangeFootprintAction>("ChangeFootprint", builder);
}