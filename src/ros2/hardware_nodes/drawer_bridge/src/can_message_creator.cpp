#include "drawer_bridge/can_message_creator.hpp"

namespace drawer_bridge
{

  can_msgs::msg::Frame CanMessageCreator::create_can_msg_drawer_unlock(const DrawerAddress& msg) const
  {
    robast_can_msgs::CanMessage can_msg_drawer_lock = _can_db.can_messages.at(robast_can_msgs::can_msg::DRAWER_UNLOCK);

    std::vector<robast_can_msgs::CanSignal> can_signals_drawer_lock = can_msg_drawer_lock.get_can_signals();

    can_signals_drawer_lock.at(robast_can_msgs::can_signal::id::drawer_unlock::MODULE_ID).set_data(msg.module_id);
    can_signals_drawer_lock.at(robast_can_msgs::can_signal::id::drawer_unlock::DRAWER_ID).set_data(msg.drawer_id);

    can_msg_drawer_lock.set_can_signals(can_signals_drawer_lock);

    return _can_encoder_decoder.encode_msg(can_msg_drawer_lock);
  }

  can_msgs::msg::Frame CanMessageCreator::create_can_msg_led_header(const uint32_t module_id,
                                                                    const uint16_t start_index,
                                                                    const uint16_t num_of_leds,
                                                                    const uint8_t fade_time_in_hundreds_of_ms) const
  {
    robast_can_msgs::CanMessage can_msg_led_header = _can_db.can_messages.at(robast_can_msgs::can_msg::LED_HEADER);

    std::vector<robast_can_msgs::CanSignal> can_signals_led_header = can_msg_led_header.get_can_signals();

    can_signals_led_header.at(robast_can_msgs::can_signal::id::led_header::MODULE_ID).set_data(module_id);
    can_signals_led_header.at(robast_can_msgs::can_signal::id::led_header::START_INDEX).set_data(start_index);
    can_signals_led_header.at(robast_can_msgs::can_signal::id::led_header::NUM_OF_LEDS).set_data(num_of_leds);
    can_signals_led_header.at(robast_can_msgs::can_signal::id::led_header::FADE_TIME_IN_HUNDREDS_OF_MS)
      .set_data((uint8_t) (fade_time_in_hundreds_of_ms));

    can_msg_led_header.set_can_signals(can_signals_led_header);

    return _can_encoder_decoder.encode_msg(can_msg_led_header);
  }

  can_msgs::msg::Frame CanMessageCreator::create_can_msg_set_led_state(const Led& led_state,
                                                                       const uint32_t module_id,
                                                                       const bool is_group_state,
                                                                       const bool ack_requested) const
  {
    robast_can_msgs::CanMessage can_msg_set_single_led_state =
      _can_db.can_messages.at(robast_can_msgs::can_msg::LED_STATE);

    std::vector<robast_can_msgs::CanSignal> can_signals_single_led_state =
      can_msg_set_single_led_state.get_can_signals();

    can_signals_single_led_state.at(robast_can_msgs::can_signal::id::led_state::MODULE_ID).set_data(module_id);
    can_signals_single_led_state.at(robast_can_msgs::can_signal::id::led_state::LED_STATE_RED).set_data(led_state.red);
    can_signals_single_led_state.at(robast_can_msgs::can_signal::id::led_state::LED_STATE_GREEN)
      .set_data(led_state.green);
    can_signals_single_led_state.at(robast_can_msgs::can_signal::id::led_state::LED_STATE_BLUE)
      .set_data(led_state.blue);
    can_signals_single_led_state.at(robast_can_msgs::can_signal::id::led_state::LED_STATE_BRIGHTNESS)
      .set_data(led_state.brightness);
    can_signals_single_led_state.at(robast_can_msgs::can_signal::id::led_state::IS_GROUP_STATE)
      .set_data(is_group_state);
    can_signals_single_led_state.at(robast_can_msgs::can_signal::id::led_state::ACK_REQUESTED).set_data(ack_requested);

    can_msg_set_single_led_state.set_can_signals(can_signals_single_led_state);

    return _can_encoder_decoder.encode_msg(can_msg_set_single_led_state);
  }

  can_msgs::msg::Frame CanMessageCreator::create_can_msg_drawer_task(const DrawerTask& msg) const
  {
    robast_can_msgs::CanMessage can_msg_drawer_task =
      _can_db.can_messages.at(robast_can_msgs::can_msg::ELECTRICAL_DRAWER_TASK);

    std::vector<robast_can_msgs::CanSignal> can_signals_drawer_task = can_msg_drawer_task.get_can_signals();

    can_signals_drawer_task.at(robast_can_msgs::can_signal::id::e_drawer_task::MODULE_ID)
      .set_data(msg.drawer_address.module_id);
    can_signals_drawer_task.at(robast_can_msgs::can_signal::id::e_drawer_task::DRAWER_ID)
      .set_data(msg.drawer_address.drawer_id);

    can_signals_drawer_task.at(robast_can_msgs::can_signal::id::e_drawer_task::DRAWER_TARGET_POSITION)
      .set_data(msg.target_position);
    can_signals_drawer_task.at(robast_can_msgs::can_signal::id::e_drawer_task::DRAWER_SPEED).set_data(msg.speed);
    can_signals_drawer_task.at(robast_can_msgs::can_signal::id::e_drawer_task::DRAWER_STALL_GUARD_VALUE)
      .set_data(msg.stall_guard_value);

    can_msg_drawer_task.set_can_signals(can_signals_drawer_task);

    return _can_encoder_decoder.encode_msg(can_msg_drawer_task);
  }

  can_msgs::msg::Frame CanMessageCreator::create_can_msg_tray_led_brightness(const DrawerAddress& drawer_address,
                                                                             const uint8_t led_row,
                                                                             const uint8_t brightness) const
  {
    robast_can_msgs::CanMessage can_msg_tray_led_brightness =
      _can_db.can_messages.at(robast_can_msgs::can_msg::TRAY_LED_BRIGHTNESS);

    std::vector<robast_can_msgs::CanSignal> can_signals_tray_led_brightness =
      can_msg_tray_led_brightness.get_can_signals();

    can_signals_tray_led_brightness.at(robast_can_msgs::can_signal::id::tray_led_brightness::MODULE_ID)
      .set_data(drawer_address.module_id);
    can_signals_tray_led_brightness.at(robast_can_msgs::can_signal::id::tray_led_brightness::TRAY_ID)
      .set_data(drawer_address.drawer_id);
    can_signals_tray_led_brightness.at(robast_can_msgs::can_signal::id::tray_led_brightness::TRAY_LED_ROW_INDEX)
      .set_data(led_row);
    can_signals_tray_led_brightness.at(robast_can_msgs::can_signal::id::tray_led_brightness::TRAY_LED_STATE_BRIGHNESS)
      .set_data(brightness);

    can_msg_tray_led_brightness.set_can_signals(can_signals_tray_led_brightness);

    return _can_encoder_decoder.encode_msg(can_msg_tray_led_brightness);
  }

  can_msgs::msg::Frame CanMessageCreator::create_can_msg_set_module_config(const DrawerAddress& drawer_address,
                                                                           const uint8_t config_id,
                                                                           const uint32_t config_value) const
  {
    robast_can_msgs::CanMessage can_msg_set_module_config =
      _can_db.can_messages.at(robast_can_msgs::can_msg::MODULE_CONFIG);

    std::vector<robast_can_msgs::CanSignal> can_signals_set_module_config = can_msg_set_module_config.get_can_signals();

    can_signals_set_module_config.at(robast_can_msgs::can_signal::id::module_config::MODULE_ID)
      .set_data(drawer_address.module_id);
    can_signals_set_module_config.at(robast_can_msgs::can_signal::id::module_config::CONFIG_ID).set_data(config_id);
    can_signals_set_module_config.at(robast_can_msgs::can_signal::id::module_config::CONFIG_VALUE)
      .set_data(config_value);

    can_msg_set_module_config.set_can_signals(can_signals_set_module_config);

    return _can_encoder_decoder.encode_msg(can_msg_set_module_config);
  }

  can_msgs::msg::Frame CanMessageCreator::create_can_msg_e_drawer_motor_control(
    const std::shared_ptr<const ElectricalDrawerMotorControl::Goal> motor_control_goal) const
  {
    robast_can_msgs::CanMessage can_msg_e_drawer_motor_control =
      _can_db.can_messages.at(robast_can_msgs::can_msg::ELECTRICAL_DRAWER_MOTOR_CONTROL);

    std::vector<robast_can_msgs::CanSignal> can_signals_e_drawer_motor_control =
      can_msg_e_drawer_motor_control.get_can_signals();

    can_signals_e_drawer_motor_control.at(robast_can_msgs::can_signal::id::electrical_drawer_motor_control::MODULE_ID)
      .set_data(motor_control_goal->module_address.module_id);
    can_signals_e_drawer_motor_control.at(robast_can_msgs::can_signal::id::electrical_drawer_motor_control::MOTOR_ID)
      .set_data(motor_control_goal->motor_id);
    can_signals_e_drawer_motor_control
      .at(robast_can_msgs::can_signal::id::electrical_drawer_motor_control::ENABLE_MOTOR)
      .set_data(motor_control_goal->enable_motor ? robast_can_msgs::can_data::ENABLE_MOTOR
                                                 : robast_can_msgs::can_data::DISABLE_MOTOR);

    can_msg_e_drawer_motor_control.set_can_signals(can_signals_e_drawer_motor_control);

    return _can_encoder_decoder.encode_msg(can_msg_e_drawer_motor_control);
  }

}   // namespace drawer_bridge