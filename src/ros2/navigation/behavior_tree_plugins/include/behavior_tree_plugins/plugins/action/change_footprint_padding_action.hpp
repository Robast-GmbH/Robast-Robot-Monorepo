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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CHANGE_FOOTPRINT_PADDING_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CHANGE_FOOTPRINT_PADDING_HPP_

#include <string>

#include "communication_interfaces/action/change_footprint_padding.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace nav2_behavior_tree
{

  /**
   * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::Wait
   */
  class ChangeFootprintPaddingAction : public BtActionNode<communication_interfaces::action::ChangeFootprintPadding>
  {
   public:
    /**
     * @brief A constructor for ChangeFootprintPaddingAction
     * @param xml_tag_name Name for the XML tag for this node
     * @param action_name Action name this node creates a client for
     * @param conf BT node configuration
     */
    ChangeFootprintPaddingAction(const std::string& xml_tag_name,
                                 const std::string& action_name,
                                 const BT::NodeConfiguration& conf);

    /**
     * @brief Function to perform some user-defined operation on tick
     *
     */
    void on_tick() override;

    /**
     * @brief Function to perform some user-defined operation after a timeout
     * waiting for a result that hasn't been received yet
     */
    void on_wait_for_result();

    /**
     * @brief Function to perform some user-defined operation upon successful completion of the action
     */
    BT::NodeStatus on_success() override;

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific ports
     */
    static BT::PortsList providedPorts()
    {
      return providedBasicPorts(
        {BT::InputPort<double>("footprint_padding", "new footprint padding for local and global costmap")});
    }
  };

}   // namespace nav2_behavior_tree

#endif   // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CHANGE_FOOTPRINT_PADDING_HPP_