#include "robast_drawer_controller/drawer_controller.hpp"

namespace robast_drawer_controller
{
DrawerController::DrawerController() : Node("robast_drawer_controller")
{
  this->drawer_controller_server = rclcpp_action::create_server<ControlDrawer>(
    this,
    "control_drawer",
    std::bind(&DrawerController::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&DrawerController::cancel_callback, this, std::placeholders::_1),
    std::bind(&DrawerController::accepted_callback, this, std::placeholders::_1));
}

//TODO: Dekonstruktor

rclcpp_action::GoalResponse DrawerController::goal_callback(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ControlDrawer::Goal> goal)
{
  // RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
  // (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DrawerController::cancel_callback(
  const std::shared_ptr<GoalHandleControlDrawer> goal_handle)
{
  // RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  // (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DrawerController::accepted_callback(const std::shared_ptr<GoalHandleControlDrawer> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&DrawerController::open_drawer, this, std::placeholders::_1), goal_handle}.detach();
}

void DrawerController::open_drawer(const std::shared_ptr<GoalHandleControlDrawer> goal_handle) {
  //RCLCPP_INFO(this->get_logger(), "Executing goal");
}




}  // namespace robast_drawer_controller
