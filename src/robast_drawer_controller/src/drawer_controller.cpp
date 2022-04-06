#include <inttypes.h>
#include <memory>

#include "robast_drawer_controller/drawer_controller.hpp"

namespace robast_drawer_controller
{


DrawerController::DrawerController()
: nav2_util::LifecycleNode("robast_drawer_controller", "", true)
{
  RCLCPP_INFO(get_logger(), "Creating");
}

DrawerController::~DrawerController()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_util::CallbackReturn DrawerController::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  
  action_server_ = std::make_unique<ActionServer>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "drawer_controller", std::bind(&DrawerController::open_drawer, this), false);

  RCLCPP_INFO(get_logger(), "End of Configuring");

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn DrawerController::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  action_server_->activate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn DrawerController::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn DrawerController::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  action_server_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn DrawerController::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void DrawerController::open_drawer()
{
  auto goal = action_server_->get_current_goal();
  auto feedback = std::make_shared<ActionT::Feedback>();
  auto result = std::make_shared<ActionT::Result>();

  send_succeeded_action_result(goal, result);
}

void DrawerController::send_succeeded_action_result(
  const std::shared_ptr<const typename ActionT::Goal> goal,
  std::shared_ptr<ActionT::Result> result)
{
  result->success = true;
  action_server_->succeeded_current(result);
}

} // namespace robast_drawer_controller