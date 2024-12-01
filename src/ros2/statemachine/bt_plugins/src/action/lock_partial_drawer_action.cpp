// File: lock_partial_drawer_action.cpp

#include "bt_plugins/action/lock_partial_drawer_action.hpp"

namespace statemachine
{

  LockPartialDrawer::LockPartialDrawer(const std::string &name, const BT::NodeConfig &config)
      : BT::StatefulActionNode(name, config), _action_name("/motor_control")
  {
    _node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

    // Initialize the action client
    _lock_action_client = rclcpp_action::create_client<communication_interfaces::action::ElectricalDrawerMotorControl>(
        _node, _action_name);
  }

  BT::NodeStatus LockPartialDrawer::onStart()
  {
    // Wait for the action server to be available
    if (!_lock_action_client->wait_for_action_server(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(_node->get_logger(), "Action server not available");
      return BT::NodeStatus::FAILURE;
    }

    // Create a goal message
    communication_interfaces::action::ElectricalDrawerMotorControl::Goal goal_msg;

    // Get the drawer address from the input port
    if (!getInput<communication_interfaces::msg::DrawerAddress>("module_address", goal_msg.module_address))
    {
      RCLCPP_ERROR(_node->get_logger(), "Missing required input [module_address]");
      return BT::NodeStatus::FAILURE;
    }
    if (!getInput<bool>("lock", goal_msg.enable_motor))
    {
      RCLCPP_ERROR(_node->get_logger(), "Missing required input [lock]");
      return BT::NodeStatus::FAILURE;
    }
    goal_msg.motor_id = 0;

    // Send the goal asynchronously
    _goal_future = _lock_action_client->async_send_goal(goal_msg);

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus LockPartialDrawer::onRunning()
  {
    if (!_goal_handle)
    {
      // Check if the goal has been accepted
      auto status = _goal_future.wait_for(std::chrono::milliseconds(0));
      if (status == std::future_status::ready)
      {
        _goal_handle = _goal_future.get();
        if (!_goal_handle)
        {
          RCLCPP_ERROR(_node->get_logger(), "Goal was rejected by the action server");
          return BT::NodeStatus::FAILURE;
        }

        // Start getting the result
        _result_future = _lock_action_client->async_get_result(_goal_handle);
      }
      else
      {
        // Goal not yet accepted, keep waiting
        return BT::NodeStatus::RUNNING;
      }
    }

    if (_result_future.valid())
    {
      // Check if the result is ready
      auto result_status = _result_future.wait_for(std::chrono::milliseconds(0));
      if (result_status == std::future_status::ready)
      {
        auto result = _result_future.get();

        // Check the result code and return the corresponding node status
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
          RCLCPP_INFO(_node->get_logger(), "Drawer locked successfully");
          return BT::NodeStatus::SUCCESS;
        }
        else
        {
          RCLCPP_ERROR(_node->get_logger(), "Failed to lock drawer");
          return BT::NodeStatus::FAILURE;
        }
      }
    }

    // Action is still running
    return BT::NodeStatus::RUNNING;
  }

  void LockPartialDrawer::onHalted()
  {
    if (_goal_handle)
    {
      // Cancel the goal if possible
      _lock_action_client->async_cancel_goal(_goal_handle);
      _goal_handle.reset();
    }

    // Reset futures
    _goal_future = {};
    _result_future = {};
  }

}   // namespace statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<statemachine::LockPartialDrawer>("LockPartialDrawer");
}