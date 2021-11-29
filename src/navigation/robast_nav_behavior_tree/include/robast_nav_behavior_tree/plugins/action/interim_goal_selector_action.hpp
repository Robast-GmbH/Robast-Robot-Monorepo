#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__INTERIM_GOAL_SELECTOR_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__INTERIM_GOAL_SELECTOR_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "robast_msgs/action/compute_interim_goal.hpp"


namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps robast_msgs::action::ComputeInterimGoal
 */
class InterimGoalCompAction : public BtActionNode<robast_msgs::action::ComputeInterimGoal>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::WaitAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  InterimGoalCompAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   * 
   */
  void on_tick() override;

  /**
   * @brief Function to perform some user-defined operation after a timeout
   * waiting for a result that hasn't been received yet
   */
  void on_wait_for_result() override;

   /**
   * @brief Function to perform some user-defined operation upon successful completion of the action
   */
  BT::NodeStatus on_success() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("interim_pose", "Interim goal selected by the interim_goal_selector node"),
        BT::InputPort<nav_msgs::msg::Path>("path", "Path to Goal"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("poses", "Interim poses list"),
      });
  }
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__INTERIM_GOAL_SELECTOR_ACTION_HPP_