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

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config)
  {
    return std::make_unique<nav2_behavior_tree::ChangeFootprintAction>(name, "change_footprint", config);
  };

  factory.registerBuilder<nav2_behavior_tree::ChangeFootprintAction>("ChangeFootprint", builder);
}