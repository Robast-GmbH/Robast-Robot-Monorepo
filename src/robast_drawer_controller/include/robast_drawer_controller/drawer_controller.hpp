#ifndef ROBAST_DRAWER_CONTROLLER__DRAWER_CONTROLLER_HPP_
#define ROBAST_DRAWER_CONTROLLER__DRAWER_CONTROLLER_HPP_

#include <inttypes.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// C library headers
#include <stdio.h>
#include <string.h>
// Linux headers for serial communication
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include "robast_hardware_nodes_msgs/action/control_drawer.hpp"

namespace robast_drawer_controller
{
class DrawerController : public rclcpp::Node
{
public:
  using ControlDrawer = robast_hardware_nodes_msgs::action::ControlDrawer;
  using GoalHandleControlDrawer = rclcpp_action::ServerGoalHandle<ControlDrawer>;

  /**
   * @brief A constructor for robast_drawer_controller::DrawerController class
   */
  DrawerController();
  /**
   * @brief A destructor for robast_drawer_controller::DrawerController class
   */
  // ~DrawerController();
  

private:
  rclcpp_action::Server<ControlDrawer>::SharedPtr drawer_controller_server;

  rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ControlDrawer::Goal> goal);

  rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<GoalHandleControlDrawer> goal_handle);

  void accepted_callback(const std::shared_ptr<GoalHandleControlDrawer> goal_handle);

  /**
   * @brief Action server execution callback
   */
  void open_drawer(const std::shared_ptr<GoalHandleControlDrawer> goal_handle);  
};
}  // namespace robast_drawer_controller
#endif  // ROBAST_DRAWER_CONTROLLER__DRAWER_CONTROLLER_HPP_
