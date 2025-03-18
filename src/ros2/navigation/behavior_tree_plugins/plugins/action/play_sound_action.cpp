#include "behavior_tree_plugins/plugins/action/play_sound_action.hpp"

namespace nav2_behavior_tree
{

  PlaySoundAction::PlaySoundAction(const std::string& xml_tag_name,
                                   const std::string& action_name,
                                   const BT::NodeConfiguration& conf)
      : BtActionNode<communication_interfaces::action::PlaySound>(xml_tag_name, action_name, conf)
  {
    getInput("file_name", goal_.sound_file);
  }
}   // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config)
  {
    return std::make_unique<nav2_behavior_tree::PlaySoundAction>(name, "play_sound", config);
  };

  factory.registerBuilder<nav2_behavior_tree::PlaySoundAction>("PlaySoundAction", builder);
}