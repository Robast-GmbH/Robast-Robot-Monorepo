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
#include "communication_interfaces/msg/drawer_leds.hpp"
#include "communication_interfaces/msg/drawer_status.hpp"
#include "communication_interfaces/msg/drawer_task.hpp"
#include "communication_interfaces/msg/electrical_drawer_status.hpp"
#include "communication_interfaces/msg/module.hpp"
#include "communication_interfaces/srv/shelf_setup_info.hpp"
#include "drawer_bridge/can_encoder_decoder.hpp"
#include "drawer_bridge/can_message_creator.hpp"
#include "drawer_bridge/drawer_defines.h"
#include "drawer_bridge/qos_config.hpp"
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
    using DrawerLeds = communication_interfaces::msg::DrawerLeds;
    using DrawerStatus = communication_interfaces::msg::DrawerStatus;
    using ElectricalDrawerStatus = communication_interfaces::msg::ElectricalDrawerStatus;
    using ShelfSetupInfo = communication_interfaces::srv::ShelfSetupInfo;
    using CanMessage = can_msgs::msg::Frame;

    /**
     * @brief A constructor for drawer_bridge::DrawerBridge class
     */
    DrawerBridge();

    friend class TestDrawerBridge;   // this class has full access to all private and protected parts of this class

   private:
    /* VARIABLES */
    rclcpp::Service<ShelfSetupInfo>::SharedPtr shelf_setup_info_service_;
    rclcpp::Subscription<DrawerAddress>::SharedPtr open_drawer_subscription_;
    rclcpp::Subscription<DrawerTask>::SharedPtr drawer_task_subscription_;
    rclcpp::Subscription<DrawerLeds>::SharedPtr drawer_leds_subscription_;
    rclcpp::Subscription<CanMessage>::SharedPtr can_messages_subscription_;
    rclcpp::Publisher<DrawerStatus>::SharedPtr drawer_status_publisher_;
    rclcpp::Publisher<CanMessage>::SharedPtr can_messages_publisher_;
    rclcpp::Publisher<ElectricalDrawerStatus>::SharedPtr electrical_drawer_status_publisher_;

    robast_can_msgs::CanDb can_db_ = robast_can_msgs::CanDb();

    std::condition_variable cv_;
    std::mutex drawer_status_mutex_;

    CanEncoderDecoder can_encoder_decoder_ = CanEncoderDecoder();
    CanMessageCreator can_message_creator_ = CanMessageCreator();

    QoSConfig qos_config = QoSConfig();

    std::map<uint32_t, bool> drawer_to_be_refilled_by_drawer_controller_id_;

    /* FUNCTIONS */
    void open_drawer_topic_callback(const DrawerAddress& msg);

    void electrical_drawer_task_topic_callback(const DrawerTask& task);

    void drawer_leds_topic_callback(const DrawerLeds& msg);

    void setup_publishers();

    void setup_subscriptions();

    void setup_services();

    void send_can_msg(CanMessage can_message);

    void receive_can_msg_callback(CanMessage can_message);

    void publish_drawer_status(robast_can_msgs::CanMessage drawer_feedback_can_msg);

    void publish_electrical_drawer_status(robast_can_msgs::CanMessage electrical_drawer_feedback_can_msg);

    std::vector<communication_interfaces::msg::Module> get_all_mounted_modules();

    /**
     * @brief Service server execution callback
     */
    void provide_shelf_setup_info_callback(const std::shared_ptr<ShelfSetupInfo::Request> request,
                                           std::shared_ptr<ShelfSetupInfo::Response> response);
  };
}   // namespace drawer_bridge
#endif   // DRAWER_BRIDGE__DRAWER_BRIDGE_HPP_
