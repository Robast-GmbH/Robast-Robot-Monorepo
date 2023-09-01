
#ifndef BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_STATE_UPDATED_CONDITION_HPP_
#define BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_STATE_UPDATED_CONDITION_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_behavior_tree
{

  /**
   * @brief A BT::ConditionNode that returns SUCCESS when goal is
   * updated on the blackboard and FAILURE otherwise
   */
  class CheckStateCondition : public BT::ConditionNode
  {
   public:
    /**
     * @brief A constructor for nav2_behavior_tree::GoalUpdatedCondition
     * @param condition_name Name for the XML tag for this node
     * @param conf BT node configuration
     */
    CheckStateCondition(const std::string& condition_name, const BT::NodeConfiguration& conf);

    CheckStateCondition() = delete;

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
      return {BT::InputPort<std::string>("variable_name", "default", "name of the variable (bool) on the blackboard")};
    }

   private:
    std::string _name;
  };

}   // namespace nav2_behavior_tree

#endif   // BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_STATE_UPDATED_CONDITION_HPP_