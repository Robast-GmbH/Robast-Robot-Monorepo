#ifndef POSES_IMPORT_ACTION_HPP_
#define POSES_IMPORT_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "robast_msgs/action/import_yaml_poses.hpp"


namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps robast_msgs::action::ComputeInterimGoal
 */
class ImportYamlPosesAction : public BtActionNode<robast_msgs::action::ImportYamlPoses>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::WaitAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  ImportYamlPosesAction(
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
  // void on_wait_for_result() override;

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
        BT::InputPort<std::string>("yaml_name", "name of the yaml containing poses"),
        BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("poses", "poses loaded of the yaml"),
      });
  }
};

}  // namespace nav2_behavior_tree

#endif  // POSES_IMPORT_ACTION_HPP_