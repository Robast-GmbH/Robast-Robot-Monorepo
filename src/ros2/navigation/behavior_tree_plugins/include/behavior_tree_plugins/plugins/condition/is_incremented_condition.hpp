#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__VARIABLE_UPDATED_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__VARIABLE_UPDATED_CONDITION_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/condition_node.h"

namespace nav2_behavior_tree
{

  class IsIncrementedCondition : public BT::ConditionNode
  {
   public:
    IsIncrementedCondition(const std::string& condition_name, const BT::NodeConfiguration& conf);

    IsIncrementedCondition() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<std::string>("variable_name", "default", "name of the variable on the blackboard")};
    }

   private:
    std::string _name;
    int _state = 0;
  };

}   // namespace nav2_behavior_tree

#endif   // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__VARIABLE_UPDATED_CONDITION_HPP_