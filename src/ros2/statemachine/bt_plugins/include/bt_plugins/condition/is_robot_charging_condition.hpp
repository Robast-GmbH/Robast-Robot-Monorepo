#ifndef DRAWER_STATEMACHINE_IS_ROBOT_CHARGING_HPP_
#define DRAWER_STATEMACHINE_IS_ROBOT_CHARGING_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "robotnik_msgs/msg/battery_status.hpp"

namespace statemachine
{
  class IsRobotChargingCondition : public BT::ConditionNode
  {
  public:
    IsRobotChargingCondition(const std::string &name, const BT::NodeConfig &config);
    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<bool>("target_value", "false"),
          BT::InputPort<std::string>("topic", "/battery_status"),
          BT::OutputPort<double>("battery_level", "0.0")};
    }
    BT::NodeStatus tick() override;

  protected:
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  private:
    rclcpp::Node::SharedPtr _node;

    void callbackBatteryFeedback(const robotnik_msgs::msg::BatteryStatus::SharedPtr msg);
    rclcpp::CallbackGroup::SharedPtr _callback_group;
    rclcpp::Subscription<robotnik_msgs::msg::BatteryStatus>::SharedPtr _battery_status_sub;
    bool _last_message = false;
    double_t _battery_level = 0.0;
    bool _target_value = false;
    std::string _topic_name;
  };
} // namespace statemachine

#endif // DRAWER_STATEMACHINE_IS_ROBOT_CHARGING_HPP_