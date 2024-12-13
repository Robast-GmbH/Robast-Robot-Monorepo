#include "drawer_bridge/drawer_bridge.hpp"

namespace drawer_bridge
{
  DrawerBridge::DrawerBridge() : Node("drawer_bridge")
  {
    setup_subscriptions();
    setup_publishers();
    setup_action_server();
  }

  void DrawerBridge::setup_subscriptions()
  {
    _open_drawer_subscription = this->create_subscription<DrawerAddress>(
      "open_drawer",
      _qos_config.get_qos_open_drawer(),
      std::bind(&DrawerBridge::open_drawer_topic_callback, this, std::placeholders::_1));

    _drawer_task_subscription = this->create_subscription<DrawerTask>(
      "electrical_drawer_task",
      _qos_config.get_qos_open_drawer(),
      std::bind(&DrawerBridge::electrical_drawer_task_topic_callback, this, std::placeholders::_1));

    _led_cmd_subscription =
      this->create_subscription<LedCmd>("led_cmd",
                                        _qos_config.get_qos_led_cmd(),
                                        std::bind(&DrawerBridge::led_cmd_topic_callback, this, std::placeholders::_1));

    _led_cmd_safety_subscription = this->create_subscription<LedCmd>(
      "safety/led_cmd",
      _qos_config.get_qos_led_cmd(),
      std::bind(&DrawerBridge::led_cmd_safety_topic_callback, this, std::placeholders::_1));

    _tray_task_subscription = this->create_subscription<TrayTask>(
      "tray_task",
      _qos_config.get_qos_open_drawer(),
      std::bind(&DrawerBridge::tray_task_topic_callback, this, std::placeholders::_1));

    _can_messages_subscription = this->create_subscription<CanMessage>(
      "from_can_bus",
      _qos_config.get_qos_can_messages(),
      std::bind(&DrawerBridge::receive_can_msg_callback, this, std::placeholders::_1));
  }

  void DrawerBridge::setup_publishers()
  {
    _can_msg_publisher = create_publisher<can_msgs::msg::Frame>("to_can_bus", 10);

    _drawer_status_publisher = create_publisher<DrawerStatus>("drawer_is_open", _qos_config.get_qos_open_drawer());

    _electrical_drawer_status_publisher =
      create_publisher<ElectricalDrawerStatus>("electrical_drawer_status", _qos_config.get_qos_open_drawer());

    _push_to_close_triggered =
      create_publisher<std_msgs::msg::Bool>("push_to_close_triggered", _qos_config.get_qos_open_drawer());

    _error_msg_publisher = create_publisher<ErrorBaseMsg>("robast_error", _qos_config.get_qos_error_msgs());

    _heartbeat_publisher = create_publisher<Heartbeat>("heartbeat", _qos_config.get_qos_heartbeat());
  }

  void DrawerBridge::setup_action_server()
  {
    _module_config_action_server = rclcpp_action::create_server<ModuleConfig>(
      this,
      "module_config",
      std::bind(&DrawerBridge::handle_module_config_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DrawerBridge::handle_module_config_cancel, this, std::placeholders::_1),
      std::bind(&DrawerBridge::handle_module_config_accepted, this, std::placeholders::_1));

    _electrical_drawer_motor_control_action_server = rclcpp_action::create_server<ElectricalDrawerMotorControl>(
      this,
      "motor_control",
      std::bind(
        &DrawerBridge::handle_electrical_drawer_motor_control_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DrawerBridge::handle_electrical_drawer_motor_control_cancel, this, std::placeholders::_1),
      std::bind(&DrawerBridge::handle_electrical_drawer_motor_control_accepted, this, std::placeholders::_1));
  }

  void DrawerBridge::open_drawer_topic_callback(const DrawerAddress& msg)
  {
    if (msg.module_id != 0)
    {
      const CanMessage can_msg = _can_message_creator.create_can_msg_drawer_unlock(msg);
      send_can_msg(can_msg);
    }
  }

  void DrawerBridge::electrical_drawer_task_topic_callback(const DrawerTask& msg)
  {
    const CanMessage can_msg = _can_message_creator.create_can_msg_drawer_task(msg);
    send_can_msg(can_msg);
  }

  void DrawerBridge::led_cmd_topic_callback(const LedCmd& msg)
  {
    const uint32_t module_id = msg.drawer_address.module_id;
    const uint16_t num_of_leds = msg.leds.size();
    const uint8_t fade_time_in_hundreds_of_ms = msg.fade_time_in_ms / 100;

    RCLCPP_DEBUG(get_logger(),
                 "I heard from the /led_cmd topic the module id %i and the number of led states = %i",
                 msg.drawer_address.module_id,
                 num_of_leds);

    send_led_cmd_msg_to_can_bus(module_id, num_of_leds, fade_time_in_hundreds_of_ms, msg.leds, NO_ACK_REQUESTED);
  }

  void DrawerBridge::led_cmd_safety_topic_callback(const LedCmd& msg)
  {
    const uint32_t module_id = msg.drawer_address.module_id;
    const uint16_t num_of_leds = msg.leds.size();
    const uint8_t fade_time_in_hundreds_of_ms = msg.fade_time_in_ms / 100;

    // start a new thread to not block the main thread when we wait for the acknowledgment
    _led_cmd_with_ack_thread = std::move(std::jthread{&DrawerBridge::send_led_cmd_msg_to_can_bus_with_ack,
                                                      this,
                                                      module_id,
                                                      num_of_leds,
                                                      fade_time_in_hundreds_of_ms,
                                                      msg.leds});
  }

  void DrawerBridge::send_led_cmd_msg_to_can_bus_with_ack(const uint32_t module_id,
                                                          const uint16_t num_of_leds,
                                                          const uint8_t fade_time_in_hundreds_of_ms,
                                                          const std::vector<communication_interfaces::msg::Led>& leds)
  {
    send_led_cmd_msg_to_can_bus(module_id, num_of_leds, fade_time_in_hundreds_of_ms, leds, ACK_REQUESTED);

    wait_for_led_cmd_ack();

    if (_is_led_cmd_ack_received)
    {
      RCLCPP_DEBUG(get_logger(), "Sent LED command and received acknowledgment");
      _is_led_cmd_ack_received = false;
      _led_cmd_retries = 0;
      return;
    }

    ++_led_cmd_retries;
    if (_led_cmd_retries >= MAX_LED_CMD_RETRIES)
    {
      RCLCPP_ERROR(
        get_logger(), "Sent LED command but did not receive acknowledgment after %i retries.", MAX_LED_CMD_RETRIES);
      _led_cmd_retries = 0;
      return;
    }
    RCLCPP_WARN(
      get_logger(),
      "Sent LED command but did not receive acknowledgment. Sending led cmd again. This is now attempt number %i",
      _led_cmd_retries);
    send_led_cmd_msg_to_can_bus_with_ack(module_id, num_of_leds, fade_time_in_hundreds_of_ms, leds);
  }

  void DrawerBridge::send_led_cmd_msg_to_can_bus(const uint32_t module_id,
                                                 const uint16_t num_of_leds,
                                                 const uint8_t fade_time_in_hundreds_of_ms,
                                                 const std::vector<communication_interfaces::msg::Led>& leds,
                                                 const bool ack_requested)
  {
    uint16_t num_of_led_states_in_group = 1;
    uint16_t start_index = 0;

    // We want to loop through all leds and check how many consecutive leds have the same color and brightness
    // Then we want to send a message for all leds with the same color and brightness in one message
    for (uint16_t i = 0; i < num_of_leds; i++)
    {
      if (are_consecutive_leds_same(leds, i, i + 1))
      {
        ++num_of_led_states_in_group;
      }
      else
      {
        const CanMessage can_msg_led_header = _can_message_creator.create_can_msg_led_header(
          module_id, start_index, num_of_led_states_in_group, fade_time_in_hundreds_of_ms);
        send_can_msg(can_msg_led_header);

        const bool is_group_state = num_of_led_states_in_group > 1;

        const CanMessage can_msg_led_state =
          _can_message_creator.create_can_msg_set_led_state(leds[i], module_id, is_group_state, ack_requested);
        send_can_msg(can_msg_led_state);

        start_index = i + 1;
        num_of_led_states_in_group = 1;
      }
    }
  }

  bool DrawerBridge::are_consecutive_leds_same(const std::vector<communication_interfaces::msg::Led>& leds,
                                               uint16_t index_1,
                                               uint16_t index_2)
  {
    if (index_1 >= leds.size() || index_2 >= leds.size())
    {
      return false;
    }

    return leds[index_1].red == leds[index_2].red && leds[index_1].green == leds[index_2].green &&
           leds[index_1].blue == leds[index_2].blue && leds[index_1].brightness == leds[index_2].brightness;
  }

  void DrawerBridge::tray_task_topic_callback(const TrayTask& msg)
  {
    uint8_t num_of_leds = msg.led_brightness.size();

    for (uint8_t i = 0; i < num_of_leds; i++)
    {
      const CanMessage can_msg =
        _can_message_creator.create_can_msg_tray_led_brightness(msg.drawer_address, i + 1, msg.led_brightness[i]);
      send_can_msg(can_msg);
    }
  }

  void DrawerBridge::handle_drawer_status(const robast_can_msgs::CanMessage& drawer_feedback_can_msg)
  {
    std::vector<robast_can_msgs::CanSignal> can_signals = drawer_feedback_can_msg.get_can_signals();

    DrawerAddress drawer_address = DrawerAddress();
    drawer_address.module_id = can_signals.at(robast_can_msgs::can_signal::id::drawer_feedback::MODULE_ID).get_data();
    drawer_address.drawer_id = can_signals.at(robast_can_msgs::can_signal::id::drawer_feedback::DRAWER_ID).get_data();

    const bool is_endstop_switch_pushed =
      can_signals.at(robast_can_msgs::can_signal::id::drawer_feedback::IS_ENDSTOP_SWITCH_PUSHED).get_data() ==
      robast_can_msgs::can_data::SWITCH_IS_PUSHED;
    const bool is_lock_switch_pushed =
      can_signals.at(robast_can_msgs::can_signal::id::drawer_feedback::IS_LOCK_SWITCH_PUSHED).get_data() ==
      robast_can_msgs::can_data::SWITCH_IS_PUSHED;

    publish_drawer_status(drawer_address, is_endstop_switch_pushed, is_lock_switch_pushed);
  }

  void DrawerBridge::handle_e_drawer_feedback(const robast_can_msgs::CanMessage& electrical_drawer_feedback_can_msg)
  {
    std::vector<robast_can_msgs::CanSignal> can_signals = electrical_drawer_feedback_can_msg.get_can_signals();

    DrawerAddress drawer_address = DrawerAddress();
    drawer_address.module_id = can_signals.at(robast_can_msgs::can_signal::id::e_drawer_feedback::MODULE_ID).get_data();
    drawer_address.drawer_id = can_signals.at(robast_can_msgs::can_signal::id::e_drawer_feedback::DRAWER_ID).get_data();

    const bool is_push_to_close_triggered =
      can_signals.at(robast_can_msgs::can_signal::id::e_drawer_feedback::IS_PUSH_TO_CLOSE_TRIGGERED).get_data() ==
      robast_can_msgs::can_data::PUSH_TO_CLOSE_TRIGGERED;
    publish_push_to_close_triggered(is_push_to_close_triggered);

    const bool is_stall_guard_triggered =
      can_signals.at(robast_can_msgs::can_signal::id::e_drawer_feedback::DRAWER_IS_STALL_GUARD_TRIGGERED).get_data() ==
      robast_can_msgs::can_data::STALL_GUARD_TRIGGERED;
    publish_e_drawer_status(
      drawer_address,
      can_signals.at(robast_can_msgs::can_signal::id::e_drawer_feedback::DRAWER_POSITION).get_data(),
      is_stall_guard_triggered);

    const bool is_endstop_switch_pushed =
      can_signals.at(robast_can_msgs::can_signal::id::e_drawer_feedback::IS_ENDSTOP_SWITCH_PUSHED).get_data() ==
      robast_can_msgs::can_data::SWITCH_IS_PUSHED;
    const bool is_lock_switch_pushed =
      can_signals.at(robast_can_msgs::can_signal::id::e_drawer_feedback::IS_LOCK_SWITCH_PUSHED).get_data() ==
      robast_can_msgs::can_data::SWITCH_IS_PUSHED;

    publish_drawer_status(drawer_address, is_endstop_switch_pushed, is_lock_switch_pushed);
  }

  void DrawerBridge::handle_e_drawer_motor_control_feedback(
    const robast_can_msgs::CanMessage& e_drawer_motor_control_can_msg)
  {
    RCLCPP_DEBUG(get_logger(), "Received e_drawer_motor_control feedback message!");

    std::vector<robast_can_msgs::CanSignal> can_signals = e_drawer_motor_control_can_msg.get_can_signals();

    {
      std::scoped_lock lock(_motor_control_mutex);
      _is_motor_control_change_confirmed =
        can_signals.at(robast_can_msgs::can_signal::id::electrical_drawer_motor_control::CONFIRM_CONTROL_CHANGE)
          .get_data() == robast_can_msgs::can_data::CONTROL_CHANGE_CONFIRMED;
    }

    // Please mind: Because the request to change the motor control and the confirmation are in the same msg id,
    // This callback is triggered twice:
    // 1. When the request is sent (and the confirmation bit is false)
    // 2. When the confirmation from the drawer controller is received (and the confirmation bit is true)
    // Therefore, we need to notify the waiting thread only when the confirmation bit is true
    if (_is_motor_control_change_confirmed)
    {
      RCLCPP_DEBUG(get_logger(), "Motor control change confirmed! Notifying waiting thread!");
      _motor_control_cv.notify_all();
    }
  }

  void DrawerBridge::handle_can_acknowledgment(const robast_can_msgs::CanMessage& acknowledgment_msg)
  {
    std::vector<robast_can_msgs::CanSignal> can_signals = acknowledgment_msg.get_can_signals();

    const uint16_t referenced_msg_id =
      can_signals.at(robast_can_msgs::can_signal::id::acknowledgment::REFERENCED_MSG_ID).get_data();

    switch (referenced_msg_id)
    {
      case robast_can_msgs::can_id::LED_STATE:
        handle_led_state_acknowledgment();
        break;

      default:
        break;
    }
  }

  void DrawerBridge::handle_led_state_acknowledgment()
  {
    std::scoped_lock lock(_led_cmd_ack_mutex);
    _is_led_cmd_ack_received = true;
    RCLCPP_DEBUG(get_logger(), "Received acknowledgment for LED state change!");
    _led_cmd_ack_cv.notify_all();
  }

  void DrawerBridge::publish_push_to_close_triggered(const bool is_push_to_close_triggered)
  {
    std_msgs::msg::Bool msg;
    msg.data = is_push_to_close_triggered;

    _push_to_close_triggered->publish(msg);
  }

  void DrawerBridge::publish_e_drawer_status(const DrawerAddress drawer_address,
                                             const uint8_t position,
                                             const bool is_stall_guard_triggered)
  {
    ElectricalDrawerStatus status = ElectricalDrawerStatus();
    status.drawer_address = drawer_address;
    status.position = position;
    status.is_stall_guard_triggered = is_stall_guard_triggered;

    _electrical_drawer_status_publisher->publish(status);
  }

  void DrawerBridge::publish_drawer_status(const DrawerAddress drawer_address,
                                           const bool is_endstop_switch_pushed,
                                           const bool is_lock_switch_pushed)
  {
    DrawerStatus drawer_status_msg = DrawerStatus();
    drawer_status_msg.drawer_address = drawer_address;

    if (!is_endstop_switch_pushed)
    {
      drawer_status_msg.drawer_is_open = true;
      _drawer_status_publisher->publish(drawer_status_msg);
    }

    if (!is_lock_switch_pushed && is_endstop_switch_pushed)
    {
      drawer_status_msg.drawer_is_open = false;
      _drawer_status_publisher->publish(drawer_status_msg);
    }
  }

  void DrawerBridge::publish_drawer_error_msg(const robast_can_msgs::CanMessage& drawer_error_feedback_can_msg)
  {
    std::vector<robast_can_msgs::CanSignal> can_signals = drawer_error_feedback_can_msg.get_can_signals();

    ErrorBaseMsg error_msg = ErrorBaseMsg();

    auto message_converter = MessageConverter<ERROR_CODES_TIMEOUT_DRAWER_NOT_OPENED_INTERFACE>();

    DrawerAddress drawer_address = DrawerAddress();
    drawer_address.module_id = can_signals.at(robast_can_msgs::can_signal::id::error_feedback::MODULE_ID).get_data();
    drawer_address.drawer_id = can_signals.at(robast_can_msgs::can_signal::id::error_feedback::DRAWER_ID).get_data();

    switch (can_signals.at(robast_can_msgs::can_signal::id::error_feedback::ERROR_CODE).get_data())
    {
      case robast_can_msgs::can_data::error_code::TIMEOUT_DRAWER_NOT_OPENED:
        error_msg.error_code = ERROR_CODES_TIMEOUT_DRAWER_NOT_OPENED;
        error_msg.error_description =
          "The drawer was not opened and therefore a timeout occurred. Drawer is now locked again.";
        error_msg.error_data = message_converter.messageToString(drawer_address);
        _error_msg_publisher->publish(error_msg);
        break;
      case robast_can_msgs::can_data::error_code::DRAWER_CLOSED_IN_IDLE_STATE:
        error_msg.error_code = ERROR_CODES_DRAWER_CLOSED_IN_IDLE_STATE;
        error_msg.error_description = "The drawer was closed in the idle state, which is not the intended behavior.";
        error_msg.error_data = message_converter.messageToString(drawer_address);
        _error_msg_publisher->publish(error_msg);
        break;
      case robast_can_msgs::can_data::error_code::MOTOR_DRIVER_STATE_CONTROL_NOT_SUPPORTED_BY_MODULE:
        error_msg.error_code = ERROR_CODES_MOTOR_DRIVER_CONTROL_NOT_SUPPORTED_BY_MODULE;
        error_msg.error_description = "The motor driver state control is not supported by the module.";
        error_msg.error_data = message_converter.messageToString(drawer_address);
        _error_msg_publisher->publish(error_msg);
        break;

      default:
        break;
    }
  }

  void DrawerBridge::publish_heartbeat(const robast_can_msgs::CanMessage& heartbeat_can_msg)
  {
    std::vector<robast_can_msgs::CanSignal> can_signals = heartbeat_can_msg.get_can_signals();

    Heartbeat heartbeat_msg = Heartbeat();
    heartbeat_msg.stamp = this->now();
    heartbeat_msg.id = std::to_string(can_signals.at(robast_can_msgs::can_signal::id::heartbeat::MODULE_ID).get_data());
    heartbeat_msg.interval_in_ms =
      can_signals.at(robast_can_msgs::can_signal::id::heartbeat::INTERVAL_IN_MS).get_data();

    _heartbeat_publisher->publish(heartbeat_msg);
  }

  void DrawerBridge::receive_can_msg_callback(const CanMessage& can_msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received CAN message: id:'%d' dlc:'%d' \n ", can_msg.id, can_msg.dlc);

    const std::optional<robast_can_msgs::CanMessage> decoded_msg = _can_encoder_decoder.decode_msg(can_msg);

    if (decoded_msg.has_value())
    {
      switch (can_msg.id)
      {
        case robast_can_msgs::can_id::DRAWER_FEEDBACK:
          handle_drawer_status(decoded_msg.value());
          break;
        case robast_can_msgs::can_id::ELECTRICAL_DRAWER_FEEDBACK:
          handle_e_drawer_feedback(decoded_msg.value());
          break;
        case robast_can_msgs::can_id::ERROR_FEEDBACK:
          publish_drawer_error_msg(decoded_msg.value());
          break;
        case robast_can_msgs::can_id::ELECTRICAL_DRAWER_MOTOR_CONTROL:
          handle_e_drawer_motor_control_feedback(decoded_msg.value());
          break;
        case robast_can_msgs::can_id::ACKNOWLEDGMENT:
          handle_can_acknowledgment(decoded_msg.value());
          break;
        case robast_can_msgs::can_id::HEARTBEAT:
          publish_heartbeat(decoded_msg.value());
          break;
      }
    }
  }

  void DrawerBridge::send_can_msg(const CanMessage& can_msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "Sending can message with id '%d'!\n ", can_msg.id);

    _can_msg_publisher->publish(can_msg);
  }

  rclcpp_action::GoalResponse DrawerBridge::handle_electrical_drawer_motor_control_goal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const ElectricalDrawerMotorControl::Goal> goal)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received electrical drawer motor control goal request!");

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse DrawerBridge::handle_electrical_drawer_motor_control_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ElectricalDrawerMotorControl>> goal_handle)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received electrical drawer motor control cancel request!");

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void DrawerBridge::handle_electrical_drawer_motor_control_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ElectricalDrawerMotorControl>> goal_handle)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received electrical drawer motor control accepted request!");

    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    _e_drawer_motor_control_thread = std::move(std::jthread{
      std::bind(&DrawerBridge::set_electrical_drawer_motor_control, this, std::placeholders::_1), goal_handle});
  }

  void DrawerBridge::wait_for_motor_control_change()
  {
    std::unique_lock<std::mutex> lock(_motor_control_mutex);
    _motor_control_cv.wait_for(lock,
                               MAX_WAIT_TIME_FOR_MOTOR_CONTROL_CONFIRMATION_IN_S,
                               [this]
                               {
                                 return _is_motor_control_change_confirmed;
                               });
  }

  void DrawerBridge::wait_for_led_cmd_ack()
  {
    std::unique_lock<std::mutex> lock(_motor_control_mutex);
    _led_cmd_ack_cv.wait_for(lock,
                             MAX_WAIT_TIME_FOR_LED_CMD_ACK_IN_S,
                             [this]
                             {
                               return _is_led_cmd_ack_received;
                             });
  }

  void DrawerBridge::reset_motor_control_change_flag()
  {
    std::scoped_lock lock(_motor_control_mutex);
    _is_motor_control_change_confirmed = false;
  }

  void DrawerBridge::set_electrical_drawer_motor_control(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ElectricalDrawerMotorControl>> goal_handle)
  {
    const std::shared_ptr<const ElectricalDrawerMotorControl::Goal> goal = goal_handle->get_goal();
    const CanMessage can_msg = _can_message_creator.create_can_msg_e_drawer_motor_control(goal);
    send_can_msg(can_msg);

    RCLCPP_DEBUG(this->get_logger(),
                 "Sending motor control message with module_id: '%i', motor_id: '%d' and enable_motor: '%d'",
                 goal->module_address.module_id,
                 goal->motor_id,
                 goal->enable_motor);

    wait_for_motor_control_change();

    auto result = std::make_shared<ElectricalDrawerMotorControl::Result>();
    result->success = _is_motor_control_change_confirmed;

    reset_motor_control_change_flag();

    if (rclcpp::ok())
    {
      if (!result->success)
      {
        RCLCPP_ERROR(this->get_logger(), "Setting motor control failed!");
        goal_handle->abort(result);
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Setting motor control succeeded!");
        goal_handle->succeed(result);
      }
    }
  }

  rclcpp_action::GoalResponse DrawerBridge::handle_module_config_goal(const rclcpp_action::GoalUUID& uuid,
                                                                      std::shared_ptr<const ModuleConfig::Goal> goal)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received module config goal request!");

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse DrawerBridge::handle_module_config_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ModuleConfig>> goal_handle)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received module config cancel request!");

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void DrawerBridge::handle_module_config_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ModuleConfig>> goal_handle)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received module config accepted request!");
    uint32_t module_id = goal_handle->get_goal()->module_address.module_id;
    uint8_t config_id = goal_handle->get_goal()->config_id;
    uint32_t config_value = goal_handle->get_goal()->config_value;

    _module_config_thread =
      std::move(std::jthread{std::bind(&DrawerBridge::set_module_config, this, std::placeholders::_1), goal_handle});
  }

  void DrawerBridge::set_module_config(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ModuleConfig>> goal_handle)
  {
    const uint32_t module_id = goal_handle->get_goal()->module_address.module_id;
    const uint8_t config_id = goal_handle->get_goal()->config_id;
    const uint32_t config_value = goal_handle->get_goal()->config_value;

    RCLCPP_INFO(this->get_logger(),
                "Setting module config with module_id: '%i', config_id: '%d' and config_value: '%d'",
                module_id,
                config_id,
                config_value);

    DrawerAddress drawer_address = DrawerAddress();
    drawer_address.module_id = module_id;

    const CanMessage can_msg =
      _can_message_creator.create_can_msg_set_module_config(drawer_address, config_id, config_value);
    send_can_msg(can_msg);

    // TODO: Wait for answer on CAN BUS!
    auto result = std::make_shared<ModuleConfig::Result>();
    if (rclcpp::ok())
    {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Setting module config succeeded");
    }
  }

}   // namespace drawer_bridge
