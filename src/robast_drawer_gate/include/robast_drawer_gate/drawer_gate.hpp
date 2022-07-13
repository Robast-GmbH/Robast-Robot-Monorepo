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
#include <thread>
#include <mutex>
#include <condition_variable>
#include <map>
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

  struct drawer_status
  {
    bool is_endstop_switch_1_pushed;
    bool is_lock_switch_1_pushed;
    bool is_endstop_switch_2_pushed;
    bool is_lock_switch_2_pushed;
    bool received_initial_drawer_status; // this is a flag to indicate that the drawer_status was received at least once at the beginning of the drawer access
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
      /* VARIABLES */
      rclcpp_action::Server<DrawerUserAccess>::SharedPtr drawer_gate_server;
      rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
      rclcpp::TimerBase::SharedPtr timer_ptr_;
      rclcpp::Service<ShelfSetupInfo>::SharedPtr shelf_setup_info_service;

      robast_serial::SerialHelper serial_helper = robast_serial::SerialHelper("/dev/serial/by-id/usb-Microchip_Technology__Inc._USBtin_A0211324-if00");

      robast_can_msgs::CanDb can_db = robast_can_msgs::CanDb();

      std::condition_variable cv;
      std::mutex drawer_status_mutex;

      std::map<uint32_t, drawer_status> drawer_status_by_drawer_controller_id;

      /* FUNCTIONS */
      void setup_serial_can_ubs_converter(void);

      robast_can_msgs::CanMessage create_can_msg_drawer_user_access(uint32_t drawer_controller_id, uint8_t drawer_id, led_parameters led_parameters, uint8_t can_data_open_lock);

      void send_can_msg(robast_can_msgs::CanMessage can_message, led_parameters led_parameters);

      void set_can_baudrate(robast_can_msgs::can_baudrate_usb_to_can_interface can_baudrate);

      void open_can_channel(void);

      void open_can_channel_listen_only_mode(void);

      void close_can_channel(void);

      void open_drawer(uint32_t drawer_controller_id, uint8_t drawer_id);

      void wait_until_initial_drawer_status_received(uint32_t drawer_controller_id);

      void wait_until_drawer_is_opened(uint32_t drawer_controller_id, uint8_t drawer_id);

      bool is_initial_drawer_status_received(uint32_t drawer_controller_id);

      bool is_drawer_open(uint32_t drawer_controller_id, uint8_t drawer_id);

      void handle_open_drawer(uint32_t drawer_controller_id, uint8_t drawer_id);

      void wait_until_drawer_is_closed(uint32_t drawer_controller_id, uint8_t drawer_id);

      bool is_drawer_closed(uint32_t drawer_controller_id, uint8_t drawer_id);

      void handle_closed_drawer(uint32_t drawer_controller_id, uint8_t drawer_id);

      rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DrawerUserAccess::Goal> goal);

      rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<GoalHandleDrawerUserAccess> goal_handle);

      void accepted_callback(const std::shared_ptr<GoalHandleDrawerUserAccess> goal_handle);

      void timer_callback(void);
      
      void update_drawer_status_from_can(void);

      void update_drawer_status(std::vector<robast_can_msgs::CanMessage> drawer_feedback_can_msgs);

      void provide_shelf_setup_info_callback(const std::shared_ptr<ShelfSetupInfo::Request> request, std::shared_ptr<ShelfSetupInfo::Response> response);

      /**
       * @brief Service server execution callback
       */
      void provide_shelf_setup_info(const std::shared_ptr<ShelfSetupInfo::Request> request, std::shared_ptr<ShelfSetupInfo::Response> response);

      /**
       * @brief Action server execution callback
       */
      void handle_drawer_user_access(const std::shared_ptr<GoalHandleDrawerUserAccess> goal_handle);  
  };
}  // namespace robast_drawer_gate
#endif  // ROBAST_DRAWER_GATE__DRAWER_GATE_HPP_
