#ifndef DRAWER_GATE__DRAWER_GATE_HPP_
#define DRAWER_GATE__DRAWER_GATE_HPP_

#include <chrono>
#include <inttypes.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
// C library headers
#include <stdio.h>
#include <string.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <map>
#include <queue>
// Linux headers for serial communication
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

// Used for the async timer
#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include "communication_interfaces/msg/drawer.hpp"
#include "communication_interfaces/msg/drawer_leds.hpp"
#include "communication_interfaces/msg/drawer_status.hpp"
#include "communication_interfaces/srv/shelf_setup_info.hpp"

#include "drawer_gate/drawer_defines.h"
#include "can/can_db.hpp"
#include "can/can_helper.h"
#include "serial_helper/serial_helper.h"

using namespace std::chrono_literals;

namespace drawer_gate
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
    using DrawerAddress = communication_interfaces::msg::DrawerAddress;
    using DrawerLeds = communication_interfaces::msg::DrawerLeds;
    using DrawerStatus = communication_interfaces::msg::DrawerStatus;
    using ShelfSetupInfo = communication_interfaces::srv::ShelfSetupInfo;

    /**
     * @brief A constructor for drawer_gate::DrawerGate class
     */
    DrawerGate(const string serial_path = "/dev/robast/robast_can");
    /**
     * @brief A destructor for drawer_gate::DrawerGate class
     */
    ~DrawerGate();

    friend class TestDrawerGate; // this class has full access to all private and protected parts of this class

  private:
    /* VARIABLES */
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
    rclcpp::TimerBase::SharedPtr timer_ptr_;
    rclcpp::Service<ShelfSetupInfo>::SharedPtr shelf_setup_info_service_;
    rclcpp::Subscription<DrawerAddress>::SharedPtr open_drawer_subscription_;
    rclcpp::Subscription<DrawerLeds>::SharedPtr drawer_leds_subscription_;
    rclcpp::Publisher<DrawerStatus>::SharedPtr drawer_status_publisher_;
    rclcpp::TimerBase::SharedPtr send_ascii_cmds_timer_;

    serial_helper::ISerialHelper* serial_helper_;

    robast_can_msgs::CanDb can_db_ = robast_can_msgs::CanDb();

    std::condition_variable cv_;
    std::mutex drawer_status_mutex_;

    std::map<uint32_t, drawer_status> drawer_status_by_drawer_controller_id_;

    queue<string> ascii_cmd_queue_; // queue that contains all ascii commands to be sent to the usb serial can adapter to make sure there is enough time between each ascii command, otherwise some commands might get lost

    bool cleared_serial_buffer_from_old_can_msgs_; // flag, that is responsible for clearing the serial buffer from old CAN messages

    std::map<uint32_t, bool> drawer_to_be_refilled_by_drawer_controller_id_;

    /* FUNCTIONS */
    void open_drawer_topic_callback(const DrawerAddress& msg);

    void drawer_leds_topic_callback(const DrawerLeds& msg);

    void setup_serial_can_ubs_converter(void);

    robast_can_msgs::CanMessage create_can_msg_drawer_lock(uint32_t drawer_controller_id, uint8_t drawer_id, uint8_t can_data_open_lock) const;

    robast_can_msgs::CanMessage create_can_msg_drawer_led(uint32_t drawer_controller_id, led_parameters led_parameters) const;

    void send_can_msg(robast_can_msgs::CanMessage can_message);

    void set_can_baudrate(robast_can_msgs::can_baudrate_usb_to_can_interface can_baudrate);

    void open_can_channel(void);

    void open_can_channel_listen_only_mode(void);

    void close_can_channel(void);

    void wait_until_initial_drawer_status_received(uint32_t drawer_controller_id);

    void wait_until_drawer_is_opened(uint32_t drawer_controller_id, uint8_t drawer_id);

    bool is_initial_drawer_status_received(uint32_t drawer_controller_id);

    bool is_drawer_open(uint32_t drawer_controller_id, uint8_t drawer_id);

    void handle_open_drawer(uint32_t drawer_controller_id, uint8_t drawer_id);

    void wait_until_drawer_is_closed(uint32_t drawer_controller_id, uint8_t drawer_id);

    bool is_drawer_closed(uint32_t drawer_controller_id, uint8_t drawer_id);

    void handle_default_drawer_status(uint32_t drawer_controller_id);

    void send_default_led_status_to_drawer(uint32_t drawer_controller_id);

    void handle_default_led_status_for_all_drawers();

    void timer_callback(void);

    void add_ascii_cmd_to_queue(std::string ascii_cmd);

    void send_ascii_cmds_timer_callback(void);

    void update_drawer_status_from_can(void);

    void update_drawer_status(std::vector<robast_can_msgs::CanMessage> drawer_feedback_can_msgs);

    std::vector<communication_interfaces::msg::Drawer> get_all_mounted_drawers();

    /**
     * @brief Service server execution callback
     */
    void provide_shelf_setup_info_callback(const std::shared_ptr<ShelfSetupInfo::Request> request, std::shared_ptr<ShelfSetupInfo::Response> response);
  };
}  // namespace drawer_gate
#endif  // DRAWER_GATE__DRAWER_GATE_HPP_
