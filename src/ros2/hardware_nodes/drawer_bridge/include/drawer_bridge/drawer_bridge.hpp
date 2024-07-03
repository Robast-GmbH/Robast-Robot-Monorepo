#ifndef DRAWER_BRIDGE__DRAWER_BRIDGE_HPP_
#define DRAWER_BRIDGE__DRAWER_BRIDGE_HPP_

#include <inttypes.h>

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
// C library headers
#include <stdio.h>
#include <string.h>

#include <condition_variable>
#include <map>
#include <mutex>
#include <queue>
#include <thread>
// Linux headers for serial communication
#include <errno.h>     // Error integer and strerror() function
#include <fcntl.h>     // Contains file controls like O_RDWR
#include <termios.h>   // Contains POSIX terminal control definitions
#include <unistd.h>    // write(), read(), close()

#include <iostream>

#include "can/can_db.hpp"
#include "can/can_helper.h"
#include "can_msgs/msg/frame.hpp"
#include "communication_interfaces/msg/drawer_status.hpp"
#include "communication_interfaces/msg/drawer_task.hpp"
#include "communication_interfaces/msg/electrical_drawer_status.hpp"
#include "communication_interfaces/msg/error_base_msg.hpp"
#include "communication_interfaces/msg/led.hpp"
#include "communication_interfaces/msg/led_cmd.hpp"
#include "communication_interfaces/msg/module.hpp"
#include "communication_interfaces/msg/tray_task.hpp"
#include "communication_interfaces/srv/shelf_setup_info.hpp"
#include "drawer_bridge/can_encoder_decoder.hpp"
#include "drawer_bridge/can_message_creator.hpp"
#include "drawer_bridge/drawer_defines.h"
#include "drawer_bridge/qos_config.hpp"
#include "error_utils/error_definitions.hpp"
#include "error_utils/generic_error_converter.hpp"
#include "shelf_setup.hpp"

using namespace std::chrono_literals;

namespace drawer_bridge
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
    bool is_endstop_switch_pushed;
    bool is_lock_switch_pushed;
  };

  struct electric_drawer_status : public drawer_status
  {
    bool is_stall_guard_triggered;
    int drawer_position;
  };

  class DrawerBridge : public rclcpp::Node
  {
   public:
    using DrawerAddress = communication_interfaces::msg::DrawerAddress;
    using DrawerTask = communication_interfaces::msg::DrawerTask;
    using Led = communication_interfaces::msg::Led;
    using LedCmd = communication_interfaces::msg::LedCmd;
    using DrawerStatus = communication_interfaces::msg::DrawerStatus;
    using ElectricalDrawerStatus = communication_interfaces::msg::ElectricalDrawerStatus;
    using ErrorBaseMsg = communication_interfaces::msg::ErrorBaseMsg;
    using ShelfSetupInfo = communication_interfaces::srv::ShelfSetupInfo;
    using TrayTask = communication_interfaces::msg::TrayTask;
    using CanMessage = can_msgs::msg::Frame;

    /**
     * @brief A constructor for drawer_bridge::DrawerBridge class
     */
    DrawerBridge();

    friend class TestDrawerBridge;   // this class has full access to all private and protected parts of this class

   private:
    /* VARIABLES */
    rclcpp::Service<ShelfSetupInfo>::SharedPtr _shelf_setup_info_service;
    rclcpp::Subscription<DrawerAddress>::SharedPtr _open_drawer_subscription;
    rclcpp::Subscription<DrawerTask>::SharedPtr _drawer_task_subscription;
    rclcpp::Subscription<LedCmd>::SharedPtr _led_cmd_subscription;
    rclcpp::Subscription<TrayTask>::SharedPtr _tray_task_subscription;
    rclcpp::Subscription<CanMessage>::SharedPtr _can_messages_subscription;
    rclcpp::Publisher<DrawerStatus>::SharedPtr _drawer_status_publisher;
    rclcpp::Publisher<ElectricalDrawerStatus>::SharedPtr _electrical_drawer_status_publisher;
    rclcpp::Publisher<ErrorBaseMsg>::SharedPtr _error_msg_publisher;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr _can_msg_publisher;

    robast_can_msgs::CanDb _can_db = robast_can_msgs::CanDb();

    CanEncoderDecoder _can_encoder_decoder = CanEncoderDecoder();
    CanMessageCreator _can_message_creator = CanMessageCreator();

    QoSConfig _qos_config = QoSConfig();

    /* FUNCTIONS */
    void open_drawer_topic_callback(const DrawerAddress& msg);

    void electrical_drawer_task_topic_callback(const DrawerTask& task);

    void led_cmd_topic_callback(const LedCmd& msg);

    void tray_task_topic_callback(const TrayTask& msg);

    void setup_publishers();

    void setup_subscriptions();

    void setup_services();

    void send_can_msg(CanMessage can_msg);

    void receive_can_msg_callback(CanMessage can_msg);

    void publish_drawer_status(robast_can_msgs::CanMessage drawer_feedback_can_msg);

    void publish_electrical_drawer_status(robast_can_msgs::CanMessage electrical_drawer_feedback_can_msg);

    void publish_drawer_error_msg(robast_can_msgs::CanMessage drawer_error_feedback_can_msg);

    std::vector<communication_interfaces::msg::Module> get_all_mounted_modules();

    /**
     * @brief Service server execution callback
     */
    void provide_shelf_setup_info_callback(const std::shared_ptr<ShelfSetupInfo::Request> request,
                                           std::shared_ptr<ShelfSetupInfo::Response> response);
  };
}   // namespace drawer_bridge
#endif   // DRAWER_BRIDGE__DRAWER_BRIDGE_HPP_
