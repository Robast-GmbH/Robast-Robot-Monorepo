#include <string>
#include <vector>
#include "behavior_tree_plugins/plugins/condition/check_for_handle.hpp"

namespace nav2_behavior_tree
{
DetectHandle::DetectHandle(
 const std::string & condition_name,
 const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
        getInput("handle_detected",_handle_detected);
        getInput("handle_pose",_handle_pose);
}

}

BT::NodeStatus CheckStateCondition::tick()
{
 int handle_detected = 0;
 std::string handle_pose;
 config().blackboard->get<std::string>(_handle_detected, handle_detected);
 config().blackboard->get<std::string>(_handle_pose, handle_pose);

 if (handle_detected == 0)
 {
       setOutput
       return BT::NodeStatus::SUCCESS;

 }
 else
 {
       return BT:NodeStatus::FAILURE;
 }

}