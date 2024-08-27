
#ifndef BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_STATE_UPDATED_CONDITION_HPP_
#define BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_STATE_UPDATED_CONDITION_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_behavior_tree
{

  class CheckStateCondition : public BT::ConditionNode
  {
   public:
    CheckStateCondition(const std::string& condition_name, const BT::NodeConfiguration& conf);

    CheckStateCondition() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<std::string>("variable_name", "default", "name of the variable (bool) on the blackboard")};
    }

   private:
    std::string _name;
  };

}   // namespace nav2_behavior_tree

#endif   // BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_STATE_UPDATED_CONDITION_HPP_