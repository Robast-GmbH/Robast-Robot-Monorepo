#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CHANGE_FOOTPRINT_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CHANGE_FOOTPRINT_HPP_

#include <string>

#include "communication_interfaces/action/change_footprint.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace nav2_behavior_tree
{

  class ChangeFootprintAction : public BtActionNode<communication_interfaces::action::ChangeFootprint>
  {
   public:
    ChangeFootprintAction(const std::string& xml_tag_name,
                          const std::string& action_name,
                          const BT::NodeConfiguration& conf);

    void on_tick() override;

    void on_wait_for_result();

    BT::NodeStatus on_success() override;

    static BT::PortsList providedPorts()
    {
      return providedBasicPorts(
        {BT::InputPort<std::string>("footprint", "new footprint for local and global costmap"),
         BT::InputPort<uint32_t>(
           "time_until_reset_in_ms", 5000, "time until footprint is reset to its default value")});
    }
  };

}   // namespace nav2_behavior_tree

#endif   // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CHANGE_FOOTPRINT_HPP_