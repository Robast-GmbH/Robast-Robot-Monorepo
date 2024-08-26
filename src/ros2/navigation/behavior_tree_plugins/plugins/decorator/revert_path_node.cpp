#ifndef __REVERT_PATH_H__
#define __REVERT_PATH_H__

#include "behavior_tree_plugins/plugins/decorator/revert_path_node.hpp"

#include <memory>
#include <string>

namespace nav2_behavior_tree
{

  RevertPath::RevertPath(const std::string& name, const BT::NodeConfiguration& conf) : BT::DecoratorNode(name, conf)
  {
  }

  BT::NodeStatus RevertPath::tick()
  {
    nav_msgs::msg::Path path;
    getInput("path", path);

    setStatus(BT::NodeStatus::RUNNING);

    if (path.poses.size() > 0)
    {
      nav_msgs::msg::Path reverted_path = path;
      std::vector<geometry_msgs::msg::PoseStamped> arr = path.poses;
      int n = path.poses.size();
      for (int i = 0; i < n; i++)
      {
        arr[i] = path.poses[n - 1 - i];
      }
      reverted_path.poses = arr;
      setOutput("reverted_path", reverted_path);

      const BT::NodeStatus child_state = child_node_->executeTick();

      switch (child_state)
      {
        case BT::NodeStatus::RUNNING:
          return BT::NodeStatus::RUNNING;

        case BT::NodeStatus::SUCCESS:
          return BT::NodeStatus::SUCCESS;

        case BT::NodeStatus::FAILURE:
          return BT::NodeStatus::FAILURE;

        default:
          return BT::NodeStatus::FAILURE;
      }
    }
    return BT::NodeStatus::FAILURE;
  }

}   // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RevertPath>("RevertPath");
}

#endif   // __REVERT_PATH_H__