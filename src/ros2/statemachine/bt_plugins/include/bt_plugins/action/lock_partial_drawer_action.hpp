#ifndef DRAWER_STATEMACHINE__BT_PLUGINS__ACTION__LOCKPARTIALDRAWER_BT_NODES_H
#define DRAWER_STATEMACHINE__BT_PLUGINS__ACTION__LOCKPARTIALDRAWER_BT_NODES_H

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "communication_interfaces/action/electrical_drawer_motor_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

namespace statemachine
{
  class LockPartialDrawer : public BT::StatefulActionNode
  {
   public:
    LockPartialDrawer(const std::string &name, const BT::NodeConfig &config);

    LockPartialDrawer() = delete;

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<communication_interfaces::msg::DrawerAddress>("drawer_address",
                                                                      "Address of the drawer to lock"),
          BT::InputPort<bool>("lock", "Lock the drawer"),
      };
    }

   protected:
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

   private:
    std::string _action_name;
    BT::Blackboard::Ptr _blackboard;
    rclcpp::Node::SharedPtr _node;
    rclcpp_action::Client<communication_interfaces::action::ElectricalDrawerMotorControl>::SharedPtr
        _lock_action_client;
    communication_interfaces::msg::DrawerAddress _drawer_address;
    std::shared_future<
        rclcpp_action::ClientGoalHandle<communication_interfaces::action::ElectricalDrawerMotorControl>::SharedPtr>
        _goal_future;
    std::shared_future<
        rclcpp_action::ClientGoalHandle<communication_interfaces::action::ElectricalDrawerMotorControl>::WrappedResult>
        _result_future;

    rclcpp_action::ClientGoalHandle<communication_interfaces::action::ElectricalDrawerMotorControl>::SharedPtr
        _goal_handle;
  };
}   // namespace statemachine

#endif