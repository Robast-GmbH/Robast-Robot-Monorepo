#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__VARIABLE_UPDATED_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__VARIABLE_UPDATED_CONDITION_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{

  /**
   * @brief A BT::ConditionNode that returns SUCCESS when goal is
   * updated on the blackboard and FAILURE otherwise
   */
  class VariableUpdatedCondition : public BT::ConditionNode
  {
   public:
    /**
     * @brief A constructor for nav2_behavior_tree::GoalUpdatedCondition
     * @param condition_name Name for the XML tag for this node
     * @param conf BT node configuration
     */
    VariableUpdatedCondition(const std::string& condition_name, const BT::NodeConfiguration& conf);

    VariableUpdatedCondition() = delete;

    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override;

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing node-specific ports
     */
    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<std::string>("variable_name", "default", "name of the variable on the blackboard")};
    }

   private:
    std::string _name;
    std::string _state = "false";
  };

}   // namespace nav2_behavior_tree

#endif   // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__VARIABLE_UPDATED_CONDITION_HPP_