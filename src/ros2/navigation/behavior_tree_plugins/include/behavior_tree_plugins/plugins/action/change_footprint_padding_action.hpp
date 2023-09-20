#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CHANGE_FOOTPRINT_PADDING_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CHANGE_FOOTPRINT_PADDING_HPP_

#include <string>

#include "communication_interfaces/action/change_footprint_padding.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace nav2_behavior_tree
{

  class ChangeFootprintPaddingAction : public BtActionNode<communication_interfaces::action::ChangeFootprintPadding>
  {
   public:
    ChangeFootprintPaddingAction(const std::string& xml_tag_name,
                                 const std::string& action_name,
                                 const BT::NodeConfiguration& conf);

    void on_tick() override;

    void on_wait_for_result();

    BT::NodeStatus on_success() override;

    static BT::PortsList providedPorts()
    {
      return providedBasicPorts(
        {BT::InputPort<double>("footprint_padding", "new footprint padding for local and global costmap"),
         BT::InputPort<uint32_t>(
           "time_until_reset_in_ms", 5000, "time until footprint padding is reset to its default value")});
    }
  };

}   // namespace nav2_behavior_tree

#endif   // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CHANGE_FOOTPRINT_PADDING_HPP_