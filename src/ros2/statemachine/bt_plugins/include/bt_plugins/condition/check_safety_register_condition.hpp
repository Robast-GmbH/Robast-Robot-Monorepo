#ifndef STATEMACHINE__BT_PLUGINS_CONDITION_CHECK_SAFETY_REGISTER_CONDITION_HPP
#define STATEMACHINE__BT_PLUGINS_CONDITION_CHECK_SAFETY_REGISTER_CONDITION_HPP

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "robotnik_safety_msgs/msg/register.hpp"
#include "robotnik_safety_msgs/msg/register_array.hpp"

namespace statemachine
{
  class CheckSafetyRegisterCondition : public BT::ConditionNode
  {
  public:
    CheckSafetyRegisterCondition(const std::string &name, const BT::NodeConfig &config);
    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<std::string>("key", "REAR_LASER_SAFE_ZONE_FREE"),
          BT::InputPort<std::string>("topic", "/robot/safety_module/raw_registers")};
    }
    BT::NodeStatus tick() override;

  protected:
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  private:
    rclcpp::Node::SharedPtr _node;

    void callbackSafetyRegisterFeedback(const robotnik_safety_msgs::msg::RegisterArray::SharedPtr msg);
    rclcpp::CallbackGroup::SharedPtr _callback_group;
    rclcpp::Subscription<robotnik_safety_msgs::msg::RegisterArray>::SharedPtr _safety_register_sub;
    bool _safety_register = false;
    std::string _topic_name;
  };
} // namespace statemachine
#endif // BT_PLUGINS_CONDITION_CHECK_SAFETY_REGISTER_CONDITION_HPP