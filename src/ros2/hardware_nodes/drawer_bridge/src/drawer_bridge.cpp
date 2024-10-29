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
    uint32_t module_id = msg.module_id;
    uint8_t drawer_id = msg.drawer_id;

    RCLCPP_INFO(
      get_logger(), "I heard from open_drawer topic the module_id: '%i' drawer_id: '%d ", module_id, drawer_id);

    if (module_id != 0)
    {
      const CanMessage can_msg = _can_message_creator.create_can_msg_drawer_unlock(msg);
      send_can_msg(can_msg);
    }
  }

  void DrawerBridge::electrical_drawer_task_topic_callback(const DrawerTask& msg)
  {
    uint32_t module_id = msg.drawer_address.module_id;
    uint8_t drawer_id = msg.drawer_address.drawer_id;
    uint8_t target_pos = msg.target_position;

    RCLCPP_INFO(get_logger(),
                "I heard from electrical_drawer_task topic the module_id: '%i' drawer_id: '%d with target position: %d",
                module_id,
                drawer_id,
                target_pos);

    const CanMessage can_msg = _can_message_creator.create_can_msg_drawer_task(msg);
    send_can_msg(can_msg);
  }

  void DrawerBridge::led_cmd_topic_callback(const LedCmd& msg)
  {
    uint16_t num_of_leds = msg.leds.size();

    RCLCPP_INFO(get_logger(),
                "I heard from the /led_cmd topic the module id %i and the number of led states = %i. The states for "
                "the first led are red = %i, green = %i, blue = %i, brightness = %i",
                msg.drawer_address.module_id,
                num_of_leds,
                msg.leds[0].red,
                msg.leds[0].green,
                msg.leds[0].blue,
                msg.leds[0].brightness);

    const CanMessage can_msg = _can_message_creator.create_can_msg_led_header(msg);
    send_can_msg(can_msg);

    for (uint16_t i = 0; i < num_of_leds; i++)
    {
      const CanMessage can_msg =
        _can_message_creator.create_can_msg_set_single_led_state(msg.leds[i], msg.drawer_address);
      send_can_msg(can_msg);
    }
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

  void DrawerBridge::publish_drawer_status(const robast_can_msgs::CanMessage drawer_feedback_can_msg)
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

  void DrawerBridge::handle_e_drawer_feedback(const robast_can_msgs::CanMessage electrical_drawer_feedback_can_msg)
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
    const robast_can_msgs::CanMessage e_drawer_motor_control_can_msg)
  {
    RCLCPP_INFO(get_logger(), "Received e_drawer_motor_control feedback message!");

    std::vector<robast_can_msgs::CanSignal> can_signals = e_drawer_motor_control_can_msg.get_can_signals();

    {
      std::lock_guard<std::mutex> lock(_motor_control_mutex);
      _is_motor_control_change_confirmed =
        can_signals.at(robast_can_msgs::can_signal::id::electrical_drawer_motor_control::CONFIRM_CONTROL_CHANGE)
          .get_data() == robast_can_msgs::can_data::CONTROL_CHANGE_CONFIRMED;
    }
    _motor_control_cv.notify_one();
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

  void DrawerBridge::publish_drawer_error_msg(const robast_can_msgs::CanMessage drawer_error_feedback_can_msg)
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

  void DrawerBridge::receive_can_msg_callback(CanMessage can_msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received CAN message: id:'%d' dlc:'%d' \n ", can_msg.id, can_msg.dlc);

    const std::optional<robast_can_msgs::CanMessage> decoded_msg = _can_encoder_decoder.decode_msg(can_msg);

    if (decoded_msg.has_value())
    {
      switch (can_msg.id)
      {
        case robast_can_msgs::can_id::DRAWER_FEEDBACK:
          publish_drawer_status(decoded_msg.value());
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
      }
    }
  }

  void DrawerBridge::send_can_msg(const CanMessage can_msg)
  {
    RCLCPP_INFO(this->get_logger(), "Sending can message with id '%d'!\n ", can_msg.id);

    _can_msg_publisher->publish(can_msg);
  }

  rclcpp_action::GoalResponse DrawerBridge::handle_electrical_drawer_motor_control_goal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const ElectricalDrawerMotorControl::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received electrical drawer motor control goal request!");

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse DrawerBridge::handle_electrical_drawer_motor_control_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ElectricalDrawerMotorControl>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received electrical drawer motor control cancel request!");

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void DrawerBridge::handle_electrical_drawer_motor_control_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ElectricalDrawerMotorControl>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received electrical drawer motor control accepted request!");

    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&DrawerBridge::set_electrical_drawer_motor_control, this, std::placeholders::_1), goal_handle}
      .detach();
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

  void DrawerBridge::reset_motor_control_change_flag()
  {
    std::lock_guard<std::mutex> lock(_motor_control_mutex);
    _is_motor_control_change_confirmed = false;
  }

  void DrawerBridge::set_electrical_drawer_motor_control(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ElectricalDrawerMotorControl>> goal_handle)
  {
    const std::shared_ptr<const ElectricalDrawerMotorControl::Goal> goal = goal_handle->get_goal();
    const CanMessage can_msg = _can_message_creator.create_can_msg_e_drawer_motor_control(goal);
    send_can_msg(can_msg);

    RCLCPP_INFO(this->get_logger(),
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
    RCLCPP_INFO(this->get_logger(), "Received module config goal request!");

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse DrawerBridge::handle_module_config_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ModuleConfig>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received module config cancel request!");

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void DrawerBridge::handle_module_config_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ModuleConfig>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received module config accepted request!");
    uint32_t module_id = goal_handle->get_goal()->module_address.module_id;
    uint8_t config_id = goal_handle->get_goal()->config_id;
    uint32_t config_value = goal_handle->get_goal()->config_value;

    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&DrawerBridge::set_module_config, this, std::placeholders::_1), goal_handle}.detach();
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
