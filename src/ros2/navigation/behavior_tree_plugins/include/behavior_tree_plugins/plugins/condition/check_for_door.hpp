#ifndef BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_FOR_DOOR_HPP_
#define BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_FOR_DOOR_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS when goal is
 * updated on the blackboard and FAILURE otherwise
 */
class CheckForDoor : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::GoalUpdatedCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  CheckForDoor(
    const bool & condition_name,
    const BT::NodeConfiguration & conf);

  CheckForDoor() = delete;

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
    return 
    {
      BT::InputPort<std::string>("door_detected", "default", "name of the variable that holds class id")
    };
  }

private:
  std::string _door_detected;
};

}  // namespace nav2_behavior_tree

#endif 