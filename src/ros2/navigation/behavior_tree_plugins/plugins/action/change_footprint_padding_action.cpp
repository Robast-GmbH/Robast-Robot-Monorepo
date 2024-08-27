#include "behavior_tree_plugins/plugins/action/change_footprint_padding_action.hpp"

namespace nav2_behavior_tree
{

  ChangeFootprintPaddingAction::ChangeFootprintPaddingAction(const std::string& xml_tag_name,
                                                             const std::string& action_name,
                                                             const BT::NodeConfiguration& conf)
      : BtActionNode<communication_interfaces::action::ChangeFootprintPadding>(xml_tag_name, action_name, conf)
  {
    getInput("footprint_padding", goal_.footprint_padding);
    getInput("time_until_reset_in_ms", goal_.time_until_reset_in_ms);
  }

  void ChangeFootprintPaddingAction::on_tick()
  {
  }

  BT::NodeStatus ChangeFootprintPaddingAction::on_success()
  {
    return BT::NodeStatus::SUCCESS;
  }

}   // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config)
  {
    return std::make_unique<nav2_behavior_tree::ChangeFootprintPaddingAction>(name, "change_footprint_padding", config);
  };

  factory.registerBuilder<nav2_behavior_tree::ChangeFootprintPaddingAction>("ChangeFootprintPadding", builder);
}