#ifndef __YAML_POSES_IMPORT_ACTION_H__
#define __YAML_POSES_IMPORT_ACTION_H__

#include <memory>
#include <string>

#include "behavior_tree_plugins/plugins/action/yaml_poses_import_action.hpp"

namespace nav2_behavior_tree
{

ImportYamlPosesAction::ImportYamlPosesAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<communication_interfaces::action::ImportYamlPoses>(xml_tag_name, action_name, conf)
{
}

void ImportYamlPosesAction::on_tick()
{
  getInput("yaml_name", goal_.yaml_name); //goal (in the room)
}

// void ImportYamlPosesAction::on_wait_for_result()
// {
//   // Grab the new yaml_name
//   std::string new_yaml_name;
//   getInput("yaml_name", new_yaml_name);

//   // Check if it is not same with the current one
//   if (goal_.yaml_name != new_yaml_name) {
//     // the action server on the next loop iteration
//     goal_.yaml_name = new_yaml_name;
//     goal_updated_ = true;
//   }
// }

BT::NodeStatus ImportYamlPosesAction::on_success()
{
  setOutput("poses", result_.result->poses);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::ImportYamlPosesAction>(
        name, "yaml_poses_importer", config);
    };

  factory.registerBuilder<nav2_behavior_tree::ImportYamlPosesAction>(
    "ImportYamlPosesAction", builder);
}


#endif // __YAML_POSES_IMPORT_ACTION_H__