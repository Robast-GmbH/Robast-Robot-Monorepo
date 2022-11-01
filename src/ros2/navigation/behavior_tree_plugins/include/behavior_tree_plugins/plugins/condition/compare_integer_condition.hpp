// Copyright (c) 2021 Tobias Alscher
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

#ifndef BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_STATE_UPDATED_CONDITION_HPP_
#define BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_STATE_UPDATED_CONDITION_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS when goal is
 * updated on the blackboard and FAILURE otherwise
 */
class CompareIntegerCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::GoalUpdatedCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  CompareIntegerCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  CompareIntegerCondition() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return 
    {
      BT::InputPort<int>("first_integer", "BT-Key for the first integer that is compared to the second one"),
      BT::InputPort<int>("second_integer", "BT-Key for the second integer that is compared to the first one"),
    };
  }

private:
  std::string _first_integer_name;
  std::string _second_integer_name;
};

}  // namespace nav2_behavior_tree

#endif  // BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_STATE_UPDATED_CONDITION_HPP_