#ifndef REVERT_PATH_DECORATOR_HPP_
#define REVERT_PATH_DECORATOR_HPP_

#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"



namespace nav2_behavior_tree
{

/**
 * @brief A BT::DecoratorNode that inverts the given path
 */
class RevertPath : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for revert path decorator node
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  RevertPath(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);



  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return
      {
        BT::InputPort<nav_msgs::msg::Path>("path", "Path to be reverted"),
        BT::OutputPort<nav_msgs::msg::Path>("reverted_path", "reverted path"),
      };
  }

  private:
  /**
   * @brief Function to perform some user-defined operation on tick
   * 
   */
    BT::NodeStatus tick() override;
};

}  // namespace nav2_behavior_tree

#endif  // REVERT_PATH_DECORATOR_HPP_