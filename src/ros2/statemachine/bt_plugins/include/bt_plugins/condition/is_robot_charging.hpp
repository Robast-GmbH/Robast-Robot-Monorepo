// TODO: @Tobi this is just theorycraft. We need a new docker including the robotnik msgs first.
#ifndef DRAWER_STATEMACHINE_IS_ROBOT_CHARGING_HPP_
#define DRAWER_STATEMACHINE_IS_ROBOT_CHARGING_HPP_

#include <string>
#include <memory>
#include "behaviortree_cpp/condition_node.h"
#include "robotnik_msgs/msg/battery_status.hpp"

namespace statemachine
{
  class IsRobotCharging : public BT::ConditionNode
  {
  public:
    IsRobotCharging(const std::string &name, const BT::NodeConfig &config);
    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<bool>("target_value", "false"),
          BT::InputPort<std::string>("topic", "/battery_status"),
          BT::OutputPort<float>("battery_level", "0.0")};
    }
    BT::NodeStatus tick() override;

  protected:
    void callbackBatteryFeedback(const robotnik_msgs::msg::BatteryStatus::SharedPtr msg);

  private:
    rclcpp::Node::SharedPtr _node;

    rclcpp::CallbackGroup::SharedPtr _callback_group;
    rclcpp::executors::SingleThreadedExecutor _callback_group_executor;
    rclcpp::Subscription<robotnik_msgs::msg::BatteryStatus>::SharedPtr _battery_status_sub;
    bool _last_message = false;
    bool _target_value = false;
    std::string _topic_name;
  };
} // namespace statemachine

#endif // DRAWER_STATEMACHINE_IS_ROBOT_CHARGING_HPP_