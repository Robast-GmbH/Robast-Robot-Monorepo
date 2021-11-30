#ifndef __REVERT_PATH_H__
#define __REVERT_PATH_H__

#include <memory>
#include <string>

#include "robast_nav_behavior_tree/plugins/decorator/revert_path_node.hpp"

namespace nav2_behavior_tree
{

RevertPath::RevertPath(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
}

BT::NodeStatus RevertPath::tick()
{
  nav_msgs::msg::Path path;
  getInput("path", path); 

  setStatus(BT::NodeStatus::RUNNING);

  if(path.poses.size() > 0){
    nav_msgs::msg::Path reverted_path = path;
    std::vector<geometry_msgs::msg::PoseStamped> arr = path.poses;
    int n = sizeof(path.poses) / sizeof(path.poses[0]);

    // std::reverse(path.poses, path.poses + n);
    for (int i = 0; i < n; i++)
    {
      arr[i] = path.poses[n-1-i];
    }
    
    reverted_path.poses = arr;
    // reverted_path.poses = path.poses;
    setOutput("reverted_path", reverted_path);

    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RevertPath>("RevertPath");
}


#endif // __REVERT_PATH_H__