#include "drawer/motion_controller.hpp"

namespace drawer
{
  MotionController::MotionController(const uint32_t module_id,
                                     const uint8_t id,
                                     const bool use_encoder,
                                     const uint8_t encoder_pin_a,
                                     const uint8_t encoder_pin_b,
                                     const uint8_t motor_driver_address,
                                     const std::shared_ptr<motor::EncoderConfig> encoder_config,
                                     const std::shared_ptr<motor::MotorConfig> motor_config,
                                     const std::shared_ptr<motor::MotorMonitorConfig> motor_monitor_config,
                                     const stepper_motor::StepperPinIdConfig &stepper_pin_id_config,
                                     const std::shared_ptr<interfaces::IGpioWrapper> gpio_wrapper,
                                     const std::shared_ptr<ElectricalDrawerConfig> e_drawer_config)
      : _module_id{module_id},
        _id{id},
        _encoder{std::make_shared<motor::Encoder>(use_encoder, encoder_pin_a, encoder_pin_b, encoder_config)},
        _encoder_monitor{std::make_unique<motor::EncoderMonitor>(_encoder, encoder_config)},
        _motor{std::make_shared<stepper_motor::Motor>(
          motor_driver_address, gpio_wrapper, stepper_pin_id_config, motor_config)},
        _motor_monitor{std::make_unique<motor::MotorMonitor>(_encoder, encoder_config, _motor, motor_monitor_config)},
        _gpio_wrapper{gpio_wrapper},
        _e_drawer_config{e_drawer_config}
  {
    _motor->init();
  }

  void MotionController::set_motor_driver_state(const bool enabled, const uint8_t motor_id) const
  {
    debug_printf_warning("[MotionController]: Setting motor driver state to %d\n", enabled);
    // TODO@Jacob: Once we have more then one motor, we need to change this to a switch case and use the motor_id
    enabled ? _motor->enable_driver() : _motor->disable_driver();
  }

  void MotionController::set_target_speed_and_direction(const uint8_t target_speed, const bool use_acceleration_ramp)
  {
    const uint32_t normed_target_speed_uint32 = get_normed_target_speed_uint32(target_speed);
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

  void MotionController::handle_finished_drawer_homing()
  {
    _motor->set_target_speed_instantly(TARGET_SPEED_ZERO);
    _encoder->set_current_position(DRAWER_HOMING_POSITION);
    _drawer_was_homed_once = true;
    _is_idling = true;
    debug_printf_green("[MotionController]: Drawer was homed successfully!\n");
  }

  bool MotionController::was_drawer_homed_once() const
  {
    return _drawer_was_homed_once;
  }

  bool MotionController::is_drawer_moving_out() const
  {
    return _is_drawer_moving_out;
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
    const uint8_t deceleration_distance = get_scaled_moving_out_deceleration_distance();

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

  bool MotionController::handle_finished_moving_out_drawer()
  {
    if (_encoder->get_normed_current_position() >= _target_position_uint8)
    {
      debug_printf(
        "[MotionController]: Moving e-drawer out is finished! normed_current_position_uint8: %d, "
        "_target_position_uint8: %d\n",
        _encoder->get_normed_current_position(),
        _target_position_uint8);

      _motor->set_target_speed_instantly(TARGET_SPEED_ZERO);

      _triggered_deceleration_for_drawer_moving_out = false;

      _is_idling = true;
      _timestamp_movement_finished_in_ms = millis();
      return true;
    }
    return false;
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
    else if (_encoder->get_normed_current_position() < _e_drawer_config->get_drawer_moving_in_final_homing_distance())
    {
      _motor->set_target_speed_instantly(_e_drawer_config->get_drawer_homing_speed());
    }
  }

  void MotionController::handle_finished_moving_in_drawer()
  {
    debug_printf_green("[MotionController]: Drawer is closed! Setting speed and encoder position to 0 !\n");
    _motor->set_target_speed_instantly(TARGET_SPEED_ZERO);
    _encoder->set_current_position(DRAWER_HOMING_POSITION);
    _is_drawer_opening_in_progress = false;
    _triggered_deceleration_for_drawer_moving_in = false;
  }

  void MotionController::reset_encoder()
  {
    _encoder->set_current_position(DRAWER_HOMING_POSITION);
  }

  void MotionController::set_is_idling(const bool is_idling)
  {
    _is_idling = is_idling;
  }

  bool MotionController::is_idling() const
  {
    return _is_idling;
  }

  uint8_t MotionController::get_normed_target_speed_uint8(const uint32_t target_speed) const
  {
    return (target_speed * MAX_SPEED_UINT8) / _e_drawer_config->get_drawer_max_speed();
  }

  uint8_t MotionController::get_target_speed() const
  {
    return get_normed_target_speed_uint8(_motor->get_target_speed());
  }

  uint8_t MotionController::get_current_position() const
  {
    return _encoder->get_normed_current_position();
  }

  uint8_t MotionController::get_normed_current_position() const
  {
    return _encoder->get_normed_current_position();
  }

  bool MotionController::is_drawer_moving_in() const
  {
    return !is_drawer_moving_out() && !is_idling();
  }

  bool MotionController::is_push_to_close_is_triggered() const
  {
    // if the drawer is not moving, we need to check if the drawer is pushed in
    // but make sure to wait a certain amount of time if the stall guard was triggered
    const uint32_t wait_time_in_ms_after_stall_guard_triggered =
      _e_drawer_config->get_drawer_push_in_wait_time_after_stall_guard_triggered_in_ms();
    const uint32_t wait_time_in_ms_after_movement_finished =
      _e_drawer_config->get_drawer_push_in_wait_time_after_movement_finished_in_ms();

    const bool is_wait_time_after_stall_guard_triggered_over =
      (millis() - _timestamp_stall_guard_triggered_in_ms > wait_time_in_ms_after_stall_guard_triggered);
    const bool is_wait_time_after_movement_finished_over =
      (millis() - _timestamp_movement_finished_in_ms > wait_time_in_ms_after_movement_finished);

    return is_wait_time_after_stall_guard_triggered_over && is_wait_time_after_movement_finished_over &&
           _encoder_monitor->check_if_drawer_is_pushed_in();
  }

  void MotionController::handle_stall_guard_triggered()
  {
    debug_printf_warning("[ElectricalDrawer]: Stall guard is triggered! Setting speed to 0 and creating feedback!\n");

    _motor->set_target_speed_instantly(TARGET_SPEED_ZERO);

    _motor->reset_stall_guard();

    set_is_idling(true);

    _timestamp_stall_guard_triggered_in_ms = millis();
  }

  void MotionController::handle_motor_control() const
  {
    _motor->handle_motor_control(_encoder->get_current_position());
  }

  void MotionController::update_position() const
  {
    _encoder->update_position(_motor->get_active_speed());
  }

  void MotionController::start_e_drawer_task(const utils::EDrawerTask &e_drawer_task)
  {
    set_is_idling(false);

    _target_position_uint8 = e_drawer_task.target_position;

    if (_target_position_uint8 > 0)
    {
      _is_drawer_opening_in_progress = true;
    }

    _motor->set_stall_guard(_e_drawer_config->get_use_tmc_stall_guard() ? e_drawer_task.stall_guard_value
                                                                        : STALL_GUARD_DISABLED);

    _timestamp_movement_started_in_ms = millis();

    if (e_drawer_task.is_homing)
    {
      start_homing_movement(e_drawer_task.target_speed);
    }
    else
    {
      start_normal_drawer_movement(e_drawer_task.target_speed, e_drawer_task.use_acceleration_ramp);
    }
  }

  bool MotionController::is_task_redundant(const uint8_t new_target_position) const
  {
    const uint8_t current_position = _encoder->get_normed_current_position();
    const bool is_task_redundant =
      (new_target_position == _target_position_uint8 && (!_is_idling || current_position == _target_position_uint8));
    return is_task_redundant;
  }

  bool MotionController::is_drawer_pulled_out() const
  {
    return !_is_drawer_opening_in_progress && _encoder_monitor->check_if_drawer_is_pulled_out();
  }

  bool MotionController::is_drawer_opening_in_progress() const
  {
    return _is_drawer_opening_in_progress;
  }

  uint32_t MotionController::get_normed_target_speed_uint32(const uint8_t target_speed) const
  {
    const uint32_t max_speed = _e_drawer_config->get_drawer_max_speed();
    const uint32_t target_speed_casted = static_cast<uint32_t>(target_speed);

    return (target_speed_casted * max_speed) / MAX_SPEED_UINT8;
  }

  uint8_t MotionController::get_scaled_moving_out_deceleration_distance() const
  {
    uint8_t deceleration_distance = _e_drawer_config->get_drawer_moving_out_deceleration_distance();

    const uint16_t intermediate_result = _target_position_uint8 * deceleration_distance;

    deceleration_distance = intermediate_result / UINT8_MAX;

    return deceleration_distance;
  }

  void MotionController::start_homing_movement(const uint8_t target_speed)
  {
    _motor->set_direction(stepper_motor::Direction::clockwise);
    _motor->set_target_speed_instantly(get_normed_target_speed_uint32(target_speed));
  }

  void MotionController::start_normal_drawer_movement(const uint8_t target_speed, const bool use_acceleration_ramp)
  {
    debug_printf_green(
      "[MotionController]: Starting normal drawer movement with target speed %d and target position %d!\n",
      target_speed,
      _target_position_uint8);

    _is_drawer_moving_out = _target_position_uint8 > _encoder->get_normed_current_position();

    set_target_speed_and_direction(target_speed, use_acceleration_ramp);
  }

}   // namespace drawer
