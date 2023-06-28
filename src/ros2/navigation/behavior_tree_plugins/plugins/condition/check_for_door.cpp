#include <string>
#include <vector>
#include "behavior_tree_plugins/plugins/condition/check_for_door.hpp"

namespace nav2_behavior_tree
{
  CheckForDoor::CheckForDoor(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{  
  getInput("door_detected", _door_detected);
}

BT::NodeStatus CheckForDoor::tick()
{
  int door_detected = 1;
  config().blackboard->get<int>(_door_detected, door_detected);

        if (door_detected == 0)
        {
                return BT::NodeStatus::SUCCESS;
        }
        else
        {
                return BT::NodeStatus::FAILURE;
        }

}

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::CheckForDoor>("DoorDetected");
}

