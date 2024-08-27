#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__GOAL_UPDATED_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__GOAL_UPDATED_CONDITION_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_behavior_tree
{

  class RobastGoalUpdatedCondition : public BT::ConditionNode
  {
   public:
    RobastGoalUpdatedCondition(const std::string& condition_name, const BT::NodeConfiguration& conf);

    RobastGoalUpdatedCondition() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return {};
    }

   private:
    geometry_msgs::msg::PoseStamped goal_;
    bool is_goal_updated_ = true;
  };

}   // namespace nav2_behavior_tree

#endif   // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__GOAL_UPDATED_CONDITION_HPP_