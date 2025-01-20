#include "drawer/motion_controller.hpp"

namespace drawer
{
  MotionController::MotionController(const uint32_t module_id,
                                     const uint8_t id,
                                     const std::shared_ptr<motor::Encoder> encoder,
                                     const std::shared_ptr<stepper_motor::Motor> motor,
                                     const std::shared_ptr<ElectricalDrawerConfig> e_drawer_config,
                                     const std::shared_ptr<switch_lib::Switch> endstop_switch,
                                     const std::shared_ptr<can_toolbox::CanUtils> can_utils,
                                     const std::optional<std::shared_ptr<lock::ElectricalDrawerLock>> drawer_lock,
                                     const std::shared_ptr<motor::EncoderConfig> encoder_config,
                                     const std::shared_ptr<motor::MotorMonitorConfig> motor_monitor_config)
      : _module_id{module_id},
        _id{id},
        _encoder{encoder},
        _motor{motor},
        _e_drawer_config{e_drawer_config},
        _endstop_switch{endstop_switch},
        _can_utils{can_utils},
        _drawer_lock{drawer_lock},
        _motor_monitor{std::make_unique<motor::MotorMonitor>(_encoder, encoder_config, _motor, motor_monitor_config)}
  {
  }

  void MotionController::set_target_speed_and_direction(const uint8_t target_speed, const bool use_acceleration_ramp)
  {
    uint32_t normed_target_speed_uint32 = get_normed_target_speed_uint32(target_speed);
    _is_drawer_moving_out ? _motor->set_direction(stepper_motor::Direction::counter_clockwise)
                          : _motor->set_direction(stepper_motor::Direction::clockwise);

    if (use_acceleration_ramp)
    {
      _motor->set_target_speed_with_accelerating_ramp(normed_target_speed_uint32,
                                                      _e_drawer_config->get_drawer_default_acceleration());
    }
    else
    {
      _motor->set_target_speed_instantly(normed_target_speed_uint32);
    }

    _encoder->init_encoder_before_next_movement(_is_drawer_moving_out);
  }

  void MotionController::set_target_speed_with_decelerating_ramp(const uint8_t target_speed)
  {
    _motor->set_target_speed_with_decelerating_ramp(get_normed_target_speed_uint32(target_speed),
                                                    _encoder->convert_uint8_position_to_drawer_position_scale(
                                                      _e_drawer_config->get_drawer_moving_in_deceleration_distance()),
                                                    _encoder->get_current_position());
  }

  void MotionController::start_homing_movement(const uint8_t target_speed)
  {
    _motor->set_direction(stepper_motor::Direction::clockwise);
    _motor->set_target_speed_instantly(get_normed_target_speed_uint32(target_speed));
  }

  bool MotionController::handle_initial_drawer_homing()
  {
    if (!_drawer_was_homed_once && _endstop_switch->is_switch_pressed())
    {
      _motor->set_target_speed_instantly(TARGET_SPEED_ZERO);
      _encoder->set_current_position(STALL_GUARD_DISABLED);
      _drawer_was_homed_once = true;
      debug_printf_green("[MotionController]: Drawer was homed successfully!\n");
      return true;
    }
    return false;
  }

  bool MotionController::was_drawer_homed_once() const
  {
    return _drawer_was_homed_once;
  }

  bool MotionController::is_drawer_moving_out() const
  {
    return _is_drawer_moving_out;
  }

  void MotionController::start_normal_drawer_movement(const uint8_t target_speed, const bool use_acceleration_ramp)
  {
    debug_printf_green(
      "[MotionController]: Starting normal drawer movement with target speed %d and target position %d!\n",
      target_speed,
      _target_position_uint8);

    reset_encoder_if_endstop_is_pushed();

    if (_target_position_uint8 == _encoder->get_normed_current_position())
    {
      _can_utils->enqueue_e_drawer_feedback_msg(_module_id,
                                                _id,
                                                _endstop_switch->is_switch_pressed(),
                                                LOCK_SWITCH_IS_NOT_PUSHED,
                                                is_stall_guard_triggered(),
                                                _encoder->get_normed_current_position(),
                                                PUSH_TO_CLOSE_NOT_TRIGGERED);
      return;
    }

    // We need to reset the stall guard before we start a new movement because stall guard is a status at the moment
    // TODO: "stall guard triggerd" should be an event not a status
    _can_utils->enqueue_e_drawer_feedback_msg(
      _module_id,
      _id,
      _endstop_switch->is_switch_pressed(),
      _drawer_lock.has_value() ? _drawer_lock.value()->is_lock_switch_pushed() : false,
      MOTOR_IS_NOT_STALLED,
      _encoder->get_normed_current_position(),
      PUSH_TO_CLOSE_NOT_TRIGGERED);

    _is_drawer_moving_out = _target_position_uint8 > _encoder->get_normed_current_position();

    set_target_speed_and_direction(target_speed, use_acceleration_ramp);
  }

  bool MotionController::is_stall_guard_triggered()
  {
    if (millis() - _timestamp_movement_started_in_ms <
        _e_drawer_config->get_drawer_stall_guard_wait_time_after_movement_started_in_ms())
    {
      debug_printf_warning(
        "[MotionController]: Since the movement started, %u ms passed! We need to wait %u ms until we start "
        "detecting "
        "a stall guard!\n",
        millis() - _timestamp_movement_started_in_ms,
        _e_drawer_config->get_drawer_stall_guard_wait_time_after_movement_started_in_ms());
      return false;
    }

    if (_e_drawer_config->get_use_tmc_stall_guard())
    {
      _is_tmc_stall_guard_triggered = _motor->get_is_stalled();
    }

    if (_e_drawer_config->get_use_motor_monitor_stall_guard())
    {
      _is_motor_monitor_stall_guard_triggered = _motor_monitor->is_motor_stalled();
    }

    if (!_is_tmc_stall_guard_triggered && !_is_motor_monitor_stall_guard_triggered)
    {
      return false;
    }

    if (_is_tmc_stall_guard_triggered)
    {
      debug_printf_warning("[MotionController]: Motor stall is detected by TMC stall guard!\n");
    }
    if (_is_motor_monitor_stall_guard_triggered)
    {
      debug_printf_warning("[MotionController]: Motor stall is detected by motor monitor!\n");
    }

    return true;
  }

  void MotionController::handle_decelerating_for_moving_out_drawer()
  {
    const uint8_t current_position = _encoder->get_normed_current_position();
    const uint8_t deceleration_distance = get_moving_out_deceleration_distance();

    const bool should_decelerate = (current_position + deceleration_distance) >= _target_position_uint8;
    if (!_triggered_deceleration_for_drawer_moving_out && should_decelerate)
    {
      debug_printf(
        "[MotionController]: E-drawer is moving out and will now be decelerated! normed_current_position_uint8 =%d, "
        "_target_position_uint8 = %d, deceleration_distance = %d\n",
        current_position,
        _target_position_uint8,
        deceleration_distance);
      _triggered_deceleration_for_drawer_moving_out = true;
      _motor->set_target_speed_with_decelerating_ramp(
        _e_drawer_config->get_drawer_moving_out_final_speed(),
        _encoder->convert_uint8_position_to_drawer_position_scale(deceleration_distance),
        _encoder->get_current_position());
    }
  }

  void MotionController::handle_finished_moving_out_drawer()
  {
    if (_encoder->get_normed_current_position() >= _target_position_uint8)
    {
      debug_printf(
        "[MotionController]: Moving e-drawer out is finished! normed_current_position_uint8: %d, "
        "_target_position_uint8: %d\n",
        _encoder->get_normed_current_position(),
        _target_position_uint8);

      _motor->set_target_speed_instantly(TARGET_SPEED_ZERO);

      _can_utils->enqueue_e_drawer_feedback_msg(
        _module_id,
        _id,
        _endstop_switch->is_switch_pressed(),
        _drawer_lock.has_value() ? _drawer_lock.value()->is_lock_switch_pushed() : false,
        is_stall_guard_triggered(),
        _encoder->get_normed_current_position(),
        PUSH_TO_CLOSE_NOT_TRIGGERED);

      _triggered_deceleration_for_drawer_moving_out = false;

      _is_idling = true;
      _timestamp_movement_finished_in_ms = millis();
      return;
    }
  }

  void MotionController::handle_decelerating_for_moving_in_drawer()
  {
    if (!_triggered_deceleration_for_drawer_moving_in &&
        _encoder->get_normed_current_position() < _e_drawer_config->get_drawer_moving_in_deceleration_distance())
    {
      _triggered_deceleration_for_drawer_moving_in = true;

      _motor->set_target_speed_with_decelerating_ramp(_e_drawer_config->get_drawer_homing_speed(),
                                                      _encoder->convert_uint8_position_to_drawer_position_scale(
                                                        _e_drawer_config->get_drawer_moving_in_deceleration_distance()),
                                                      _encoder->get_current_position());
    }
  }

  bool MotionController::handle_finished_moving_in_drawer()
  {
    if (_encoder->get_normed_current_position() >= _e_drawer_config->get_drawer_moving_in_final_homing_distance())
    {
      return false;
    }

    _motor->set_target_speed_instantly(_e_drawer_config->get_drawer_homing_speed());

    bool is_drawer_closed;
    if (_drawer_lock.has_value())
    {
      is_drawer_closed = _endstop_switch->is_switch_pressed() && !_drawer_lock.value()->is_lock_switch_pushed();
    }
    else
    {
      is_drawer_closed = _endstop_switch->is_switch_pressed();
    }

    _triggered_deceleration_for_drawer_moving_in = !is_drawer_closed;

    return is_drawer_closed;
  }

  void MotionController::reset_encoder_if_endstop_is_pushed()
  {
    if (_endstop_switch->is_switch_pressed())
    {
      _encoder->set_current_position(DRAWER_HOMING_POSITION);
    }
  }

  void MotionController::set_is_idling(const bool is_idling)
  {
    _is_idling = is_idling;
  }

  bool MotionController::is_idling() const
  {
    return _is_idling;
  }

  uint32_t MotionController::get_timestamp_movement_finished_in_ms() const
  {
    return _timestamp_movement_finished_in_ms;
  }

  void MotionController::set_timestamp_movement_started_in_ms(const uint32_t timestamp_movement_started_in_ms)
  {
    _timestamp_movement_started_in_ms = timestamp_movement_started_in_ms;
  }

  void MotionController::set_target_position_uint8(const uint8_t target_position_uint8)
  {
    _target_position_uint8 = target_position_uint8;
  }

  uint8_t MotionController::get_target_position_uint8() const
  {
    return _target_position_uint8;
  }

  uint8_t MotionController::get_normed_target_speed_uint8(const uint32_t target_speed) const
  {
    return (target_speed * MAX_SPEED_UINT8) / _e_drawer_config->get_drawer_max_speed();
  }

  uint8_t MotionController::get_target_speed() const
  {
    return get_normed_target_speed_uint8(_motor->get_target_speed());
  }

  uint32_t MotionController::get_normed_target_speed_uint32(const uint8_t target_speed) const
  {
    uint32_t max_speed = _e_drawer_config->get_drawer_max_speed();
    uint32_t target_speed_casted = static_cast<uint32_t>(target_speed);

    return (target_speed_casted * max_speed) / MAX_SPEED_UINT8;
  }

  uint8_t MotionController::get_moving_out_deceleration_distance() const
  {
    uint8_t deceleration_distance = _e_drawer_config->get_drawer_moving_out_deceleration_distance();

    uint16_t intermediate_result = _target_position_uint8 * deceleration_distance;

    deceleration_distance = intermediate_result / UINT8_MAX;

    return deceleration_distance;
  }

}   // namespace drawer
