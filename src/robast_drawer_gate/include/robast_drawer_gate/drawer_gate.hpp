#ifndef ROBAST_DRAWER_GATE__DRAWER_GATE_HPP_
#define ROBAST_DRAWER_GATE__DRAWER_GATE_HPP_

#include <chrono>
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

#include "robast_ros2_msgs/action/drawer_user_access.hpp"
#include "robast_ros2_msgs/srv/shelf_setup_info.hpp"

#include "robast_drawer_gate/drawer_defines.h"
#include "robast_can_msgs/can_db.h"
#include "robast_can_msgs/can_helper.h"
#include "include/robast_serial.h" //TODO: Fix that

using namespace std::chrono_literals;

namespace robast_drawer_gate
{
  struct led_parameters
  {
    uint8_t led_red;
    uint8_t led_green;
    uint8_t led_blue;
    uint8_t brightness;
    uint8_t mode;
  };

  class DrawerGate : public rclcpp::Node
  {
    public:
      using DrawerUserAccess = robast_ros2_msgs::action::DrawerUserAccess;
      using GoalHandleDrawerUserAccess = rclcpp_action::ServerGoalHandle<DrawerUserAccess>;
      using ShelfSetupInfo = robast_ros2_msgs::srv::ShelfSetupInfo;

      /**
       * @brief A constructor for robast_drawer_gate::DrawerGate class
       */
      DrawerGate();
      /**
       * @brief A destructor for robast_drawer_gate::DrawerGate class
       */
      // ~DrawerGate();
      

    private:
      rclcpp_action::Server<DrawerUserAccess>::SharedPtr drawer_gate_server;
      rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
      rclcpp::TimerBase::SharedPtr timer_ptr_;
      rclcpp::Service<ShelfSetupInfo>::SharedPtr shelf_setup_info_service;

      robast_serial::SerialHelper serial_helper = robast_serial::SerialHelper("/dev/serial/by-id/usb-Microchip_Technology__Inc._USBtin_A0211324-if00");

      robast_can_msgs::CanDb can_db = robast_can_msgs::CanDb();

      void setup_serial_can_ubs_converter(void);

      robast_can_msgs::CanMessage create_can_msg_drawer_user_access(std::shared_ptr<const DrawerUserAccess::Goal> goal, led_parameters led_parameters);

      void set_can_baudrate(robast_can_msgs::can_baudrate_usb_to_can_interface can_baudrate);

      void open_can_channel(void);

      void open_can_channel_listen_only_mode(void);

      void close_can_channel(void);

      rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DrawerUserAccess::Goal> goal);

      rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<GoalHandleDrawerUserAccess> goal_handle);

      void accepted_callback(const std::shared_ptr<GoalHandleDrawerUserAccess> goal_handle);

      void timer_callback(void);

      void provide_shelf_setup_info_callback(const std::shared_ptr<ShelfSetupInfo::Request> request, std::shared_ptr<ShelfSetupInfo::Response> response);

      /**
       * @brief Service server execution callback
       */
      void provide_shelf_setup_info(const std::shared_ptr<ShelfSetupInfo::Request> request, std::shared_ptr<ShelfSetupInfo::Response> response);

      /**
       * @brief Action server execution callback
       */
      void open_drawer(const std::shared_ptr<GoalHandleDrawerUserAccess> goal_handle);  
  };
}  // namespace robast_drawer_gate
#endif  // ROBAST_DRAWER_GATE__DRAWER_GATE_HPP_
