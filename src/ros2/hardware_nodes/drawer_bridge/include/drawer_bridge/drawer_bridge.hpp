#ifndef DRAWER_BRIDGE__DRAWER_BRIDGE_HPP_
#define DRAWER_BRIDGE__DRAWER_BRIDGE_HPP_

#include <inttypes.h>

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

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
#include "can/can_helper.hpp"
#include "can_msgs/msg/frame.hpp"
#include "communication_interfaces/action/electrical_drawer_motor_control.hpp"
#include "communication_interfaces/action/module_config.hpp"
#include "communication_interfaces/msg/drawer_status.hpp"
#include "communication_interfaces/msg/drawer_task.hpp"
#include "communication_interfaces/msg/electrical_drawer_status.hpp"
#include "communication_interfaces/msg/error_base_msg.hpp"
#include "communication_interfaces/msg/heartbeat.hpp"
#include "communication_interfaces/msg/led.hpp"
#include "communication_interfaces/msg/led_cmd.hpp"
#include "communication_interfaces/msg/tray_task.hpp"
#include "drawer_bridge/can_encoder_decoder.hpp"
#include "drawer_bridge/can_message_creator.hpp"
#include "drawer_bridge/drawer_defines.h"
#include "drawer_bridge/qos_config.hpp"
#include "error_utils/error_definitions.hpp"
#include "module_config/module_config_defines.hpp"
#include "std_msgs/msg/bool.hpp"

namespace drawer_bridge
{
  constexpr std::chrono::seconds MAX_WAIT_TIME_FOR_MOTOR_CONTROL_CONFIRMATION_IN_S = std::chrono::seconds(5);
  constexpr std::chrono::seconds MAX_WAIT_TIME_FOR_LED_CMD_ACK_IN_S = std::chrono::seconds(1);
  constexpr uint8_t MAX_LED_CMD_RETRIES = 2;
  constexpr bool ACK_REQUESTED = true;
  constexpr bool NO_ACK_REQUESTED = false;

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
    using TrayTask = communication_interfaces::msg::TrayTask;
    using Heartbeat = communication_interfaces::msg::Heartbeat;
    using CanMessage = can_msgs::msg::Frame;

    using ElectricalDrawerMotorControl = communication_interfaces::action::ElectricalDrawerMotorControl;
    using ModuleConfig = communication_interfaces::action::ModuleConfig;

    /**
     * @brief A constructor for drawer_bridge::DrawerBridge class
     */
    DrawerBridge();

    friend class TestDrawerBridge;   // this class has full access to all private and protected parts of this class

   private:
    // Publishers
    rclcpp::Publisher<ElectricalDrawerStatus>::SharedPtr _electrical_drawer_status_publisher;
    rclcpp::Publisher<DrawerStatus>::SharedPtr _drawer_status_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _push_to_close_triggered;
    rclcpp::Publisher<ErrorBaseMsg>::SharedPtr _error_msg_publisher;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr _can_msg_publisher;
    rclcpp::Publisher<Heartbeat>::SharedPtr _heartbeat_publisher;

    // Subscriptions
    rclcpp::Subscription<DrawerAddress>::SharedPtr _open_drawer_subscription;
    rclcpp::Subscription<DrawerTask>::SharedPtr _drawer_task_subscription;
    rclcpp::Subscription<LedCmd>::SharedPtr _led_cmd_subscription;
    rclcpp::Subscription<LedCmd>::SharedPtr _led_cmd_safety_subscription;
    rclcpp::Subscription<TrayTask>::SharedPtr _tray_task_subscription;
    rclcpp::Subscription<CanMessage>::SharedPtr _can_messages_subscription;

    // Action Servers
    rclcpp_action::Server<ElectricalDrawerMotorControl>::SharedPtr _electrical_drawer_motor_control_action_server;
    rclcpp_action::Server<ModuleConfig>::SharedPtr _module_config_action_server;

    robast_can_msgs::CanDb _can_db = robast_can_msgs::CanDb();

    CanEncoderDecoder _can_encoder_decoder = CanEncoderDecoder();
    CanMessageCreator _can_message_creator = CanMessageCreator();

    QoSConfig _qos_config = QoSConfig();

    std::mutex _motor_control_mutex;
    std::condition_variable _motor_control_cv;
    bool _is_motor_control_change_confirmed = false;

    std::mutex _led_cmd_ack_mutex;
    std::condition_variable _led_cmd_ack_cv;
    bool _is_led_cmd_ack_received = false;
    uint8_t _led_cmd_retries = 0;

    std::jthread _led_cmd_with_ack_thread;
    std::jthread _e_drawer_motor_control_thread;
    std::jthread _module_config_thread;

    /* FUNCTIONS */
    void open_drawer_topic_callback(const DrawerAddress& msg);

    void electrical_drawer_task_topic_callback(const DrawerTask& task);

    void led_cmd_topic_callback(const LedCmd& msg);

    void led_cmd_safety_topic_callback(const LedCmd& msg);

    void tray_task_topic_callback(const TrayTask& msg);

    void setup_publishers();

    void setup_subscriptions();

    void setup_action_server();

    void send_can_msg(const CanMessage& can_msg);

    void receive_can_msg_callback(const CanMessage& can_msg);

    void handle_drawer_status(const robast_can_msgs::CanMessage& drawer_feedback_can_msg);

    void handle_e_drawer_feedback(const robast_can_msgs::CanMessage& electrical_drawer_feedback_can_msg);

    void handle_e_drawer_motor_control_feedback(const robast_can_msgs::CanMessage& e_drawer_motor_control_can_msg);

    void handle_can_acknowledgment(const robast_can_msgs::CanMessage& acknowledgment_msg);

    void publish_drawer_error_msg(const robast_can_msgs::CanMessage& drawer_error_feedback_can_msg);

    void publish_push_to_close_triggered(const bool is_push_to_close_triggered);

    void publish_e_drawer_status(const DrawerAddress drawer_address,
                                 const uint8_t position,
                                 const bool is_stall_guard_triggered);

    void publish_drawer_status(const DrawerAddress drawer_address,
                               const bool is_endstop_switch_pushed,
                               const bool is_lock_switch_pushed);

    void handle_led_state_acknowledgment();

    void publish_heartbeat(const robast_can_msgs::CanMessage& heartbeat_msg);

    void set_module_config(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ModuleConfig>> goal_handle);

    void set_electrical_drawer_motor_control(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ElectricalDrawerMotorControl>> goal_handle);

    void wait_for_motor_control_change();

    void wait_for_led_cmd_ack();

    void reset_motor_control_change_flag();

    void send_led_cmd_msg_to_can_bus_with_ack(const uint32_t module_id,
                                              const uint16_t num_of_leds,
                                              const uint8_t fade_time_in_hundreds_of_ms,
                                              const std::vector<communication_interfaces::msg::Led>& leds);

    void send_led_cmd_msg_to_can_bus(const uint32_t module_id,
                                     const uint16_t num_of_leds,
                                     const uint8_t fade_time_in_hundreds_of_ms,
                                     const std::vector<communication_interfaces::msg::Led>& leds,
                                     const bool ack_requested);

    bool are_consecutive_leds_same(const std::vector<communication_interfaces::msg::Led>& leds,
                                   uint16_t index_1,
                                   uint16_t index_2);

    // Action Server Callbacks
    rclcpp_action::GoalResponse handle_module_config_goal(const rclcpp_action::GoalUUID& uuid,
                                                          std::shared_ptr<const ModuleConfig::Goal> goal);

    rclcpp_action::CancelResponse handle_module_config_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ModuleConfig>> goal_handle);

    void handle_module_config_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ModuleConfig>> goal_handle);

    rclcpp_action::GoalResponse handle_electrical_drawer_motor_control_goal(
      const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const ElectricalDrawerMotorControl::Goal> goal);

    rclcpp_action::CancelResponse handle_electrical_drawer_motor_control_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ElectricalDrawerMotorControl>> goal_handle);

    void handle_electrical_drawer_motor_control_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ElectricalDrawerMotorControl>> goal_handle);
  };
}   // namespace drawer_bridge
#endif   // DRAWER_BRIDGE__DRAWER_BRIDGE_HPP_
