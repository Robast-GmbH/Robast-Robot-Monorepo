#ifndef BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_STATE_UPDATED_CONDITION_HPP_
#define BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_STATE_UPDATED_CONDITION_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_behavior_tree
{

  class CompareIntegerCondition : public BT::ConditionNode
  {
   public:
    CompareIntegerCondition(const std::string& condition_name, const BT::NodeConfiguration& conf);

    CompareIntegerCondition() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<int>("first_integer", "BT-Key for the first integer that is compared to the second one"),
        BT::InputPort<int>("second_integer", "BT-Key for the second integer that is compared to the first one"),
      };
    }

   private:
    std::string _first_integer_name;
    std::string _second_integer_name;
  };

}   // namespace nav2_behavior_tree

#endif   // BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_STATE_UPDATED_CONDITION_HPP_