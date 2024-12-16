#ifndef DRAWER_STATEMACHINE_DOUBLE_COMPARE_CONDITION_HPP_
#define DRAWER_STATEMACHINE_DOUBLE_COMPARE_CONDITION_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "robotnik_msgs/msg/battery_status.hpp"

namespace statemachine
{
  class DoubleCompareCondition : public BT::ConditionNode
  {
   public:
    DoubleCompareCondition(const std::string &name, const BT::NodeConfig &config);
    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<double>("target_value", "0.0"),
              BT::InputPort<double>("value", "0.0"),
              BT::InputPort<std::string>("comparison", "==")};
    }
    BT::NodeStatus tick() override;

   private:
    double_t _target_value = 0.0;
    double_t _value = 0.0;
    std::string _comparison = "==";
  };
}   // namespace statemachine

#endif   // !DRAWER_STATEMACHINE_DOUBLE_COMPARE_CONDITION_HPP_