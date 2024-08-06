#include "drawer/electrical_drawer.hpp"

namespace drawer_controller
{
  ElectricalDrawer::ElectricalDrawer(const uint32_t module_id,
                                     const uint8_t id,
                                     const std::shared_ptr<robast_can_msgs::CanDb> can_db,
                                     const std::shared_ptr<IGpioWrapper> gpio_wrapper,
                                     const stepper_motor::StepperPinIdConfig &stepper_pin_id_config,
                                     const bool use_encoder,
                                     const uint8_t encoder_pin_a,
                                     const uint8_t encoder_pin_b,
                                     const uint8_t motor_driver_address,
                                     const std::shared_ptr<MotorConfig> motor_config,
                                     const std::shared_ptr<Switch> endstop_switch,
                                     const std::optional<std::shared_ptr<ElectricalDrawerLock>> electrical_drawer_lock,
                                     const std::shared_ptr<ElectricalDrawerConfig> e_drawer_config,
                                     const std::shared_ptr<EncoderConfig> encoder_config,
                                     const std::shared_ptr<MotorMonitorConfig> motor_monitor_config)
      : _module_id{module_id},
        _id{id},
        _gpio_wrapper{gpio_wrapper},
        _stepper_pin_id_config{stepper_pin_id_config},
        _encoder{std::make_shared<Encoder>(use_encoder, encoder_pin_a, encoder_pin_b, encoder_config)},
        _can_utils{std::make_unique<CanUtils>(can_db)},
        _motor{std::make_shared<stepper_motor::Motor>(
          motor_driver_address, _gpio_wrapper, _stepper_pin_id_config, motor_config)},
        _endstop_switch{endstop_switch},
        _electrical_drawer_lock{electrical_drawer_lock},
        _e_drawer_task_queue{std::make_unique<Queue<EDrawerTask>>()},
        _encoder_monitor{std::make_unique<EncoderMonitor>(_encoder, encoder_config)},
        _config{e_drawer_config},
        _motor_monitor{std::make_unique<MotorMonitor>(_encoder, encoder_config, _motor, motor_monitor_config)}
  {
    init();
  }

  void ElectricalDrawer::init() const
  {
    if (_electrical_drawer_lock.has_value())
    {
      _electrical_drawer_lock.value()->initialize_lock();
    }
    _motor->init();
  }

  void ElectricalDrawer::unlock()
  {
    if (_electrical_drawer_lock.has_value())
    {
      debug_println("[ElectricalDrawer]: Received request to unlock the lock!");
      _electrical_drawer_lock.value()->unlock();
    }
    else
    {
      Serial.println("[ElectricalDrawer]: Warning! Received request to unlock the lock, but no lock is installed!");
    }
  }

  std::optional<robast_can_msgs::CanMessage> ElectricalDrawer::can_out()
  {
    return _can_utils->get_element_from_feedback_msg_queue();
  }

  void ElectricalDrawer::update_state()
  {
    if (_is_idling)
    {
      handle_drawer_idle_state();
    }
    else
    {
      handle_drawer_active_state();
    }
  }

  void ElectricalDrawer::handle_drawer_idle_state()
  {
    // updating the sensor values needs to be done all the time because of the moving average calculation
    _endstop_switch->update_sensor_value();
    reset_encoder_if_endstop_is_pushed();

    // if the drawer is not moving, we need to check if the drawer is pushed in
    // but make sure to wait a certain amount of time if the stall guard was triggered
    uint32_t wait_time_in_ms = _config->get_drawer_push_in_wait_time_after_stall_guard_triggered_in_ms();
    if (millis() - _timestamp_stall_guard_triggered_in_ms > wait_time_in_ms)
    {
      if (_encoder_monitor->check_if_drawer_is_pushed_in())
      {
        add_e_drawer_task_to_queue({DRAWER_TARGET_HOMING_POSITION,
                                    _config->get_drawer_push_in_auto_close_speed(),
                                    _config->get_drawer_push_in_auto_close_stall_guard_value(),
                                    IS_NOT_HOMING,
                                    DO_NOT_USE_ACCELERATION_RAMP});
        _can_utils->enqueue_e_drawer_feedback_msg(
          _module_id,
          _id,
          _endstop_switch->is_switch_pressed(),
          _electrical_drawer_lock.has_value() ? _electrical_drawer_lock.value()->is_lock_switch_pushed() : false,
          MOTOR_IS_NOT_STALLED,
          _encoder->get_normed_current_position(),
          PUSH_TO_CLOSE_TRIGGERED);
      }
    }

    start_next_e_drawer_task();
  }

  void ElectricalDrawer::start_next_e_drawer_task()
  {
    std::optional<EDrawerTask> e_drawer_task = _e_drawer_task_queue->get_element_from_queue();
    if (e_drawer_task.has_value())
    {
      debug_printf("[ElectricalDrawer]: Received new e-drawer task from queue with target position %u and speed %u!\n",
                   e_drawer_task.value().target_position,
                   e_drawer_task.value().target_speed);

      _is_idling = false;

      _target_position_uint8 = e_drawer_task.value().target_position;

      _motor->set_stall_guard(_config->get_use_tmc_stall_guard() ? e_drawer_task.value().stall_guard_value : 0);

      _timestamp_movement_started_in_ms = millis();

      if (e_drawer_task.value().is_homing)
      {
        start_homing_movement(e_drawer_task.value().target_speed);
      }
      else
      {
        start_normal_drawer_movement(e_drawer_task.value().target_speed, e_drawer_task.value().use_acceleration_ramp);
      }
    }
  }

  void ElectricalDrawer::start_normal_drawer_movement(const uint8_t target_speed, const bool use_acceleration_ramp)
  {
    reset_encoder_if_endstop_is_pushed();

    if (_target_position_uint8 == _encoder->get_normed_current_position())
    {
      _can_utils->enqueue_e_drawer_feedback_msg(_module_id,
                                                _id,
                                                _endstop_switch->is_switch_pressed(),
                                                false,
                                                is_stall_guard_triggered(),
                                                _encoder->get_normed_current_position(),
                                                PUSH_TO_CLOSE_NOT_TRIGGERED);
      return;
    }

    _is_drawer_moving_out = _target_position_uint8 > _encoder->get_normed_current_position();

    set_target_speed_and_direction(target_speed, use_acceleration_ramp);
  }

  void ElectricalDrawer::start_homing_movement(const uint8_t target_speed)
  {
    _motor->set_direction(stepper_motor::clockwise);
    _motor->set_target_speed_instantly(get_normed_target_speed_uint32(target_speed));
  }

  void ElectricalDrawer::handle_drawer_active_state()
  {
    if (_electrical_drawer_lock.has_value())
    {
      handle_electrical_drawer_lock_control();
    }

    if (is_stall_guard_triggered())
    {
      handle_stall_guard_triggered();
      return;
    }

    _endstop_switch->update_sensor_value();

    if (check_and_handle_initial_drawer_homing())
    {
      return;
    }

    _motor->handle_motor_control(_encoder->get_current_position());

    _encoder->update_position(_motor->get_active_speed());

    _is_drawer_moving_out ? handle_drawer_moving_out() : handle_drawer_moving_in();

    debug_prints_moving_electrical_drawer();
  }

  bool ElectricalDrawer::check_and_handle_initial_drawer_homing()
  {
    if (_drawer_was_homed_once)
    {
      return false;
    }

    if (_endstop_switch->is_switch_pressed())
    {
      _motor->set_target_speed_instantly(0);
      _encoder->set_current_position(0);
      _drawer_was_homed_once = true;
      _is_idling = true;
      debug_println("[ElectricalDrawer]: Drawer was homed successfully!");
    }
    return true;
  }

  void ElectricalDrawer::handle_electrical_drawer_lock_control()
  {
    _electrical_drawer_lock.value()->update_sensor_values();

    _electrical_drawer_lock.value()->handle_lock_control();

    if (_electrical_drawer_lock.value()->is_drawer_auto_close_timeout_triggered())
    {
      _can_utils->enqueue_error_feedback_msg(_module_id, _id, CAN_DATA_ERROR_CODE_TIMEOUT_DRAWER_NOT_OPENED);
      _electrical_drawer_lock.value()->set_drawer_auto_close_timeout_triggered(false);
    }

    handle_drawer_just_opened();
  }

  bool ElectricalDrawer::is_stall_guard_triggered()
  {
    if (millis() - _timestamp_movement_started_in_ms <
        _config->get_drawer_stall_guard_wait_time_after_movement_started_in_ms())
    {
      debug_printf(
        "[ElectricalDrawer]: Since the movement started, %u ms passed! We need to wait %u ms until we start detecting "
        "a stall guard!\n",
        millis() - _timestamp_movement_started_in_ms,
        _config->get_drawer_stall_guard_wait_time_after_movement_started_in_ms());
      return false;
    }

    if (_config->get_use_tmc_stall_guard())
    {
      _is_tmc_stall_guard_triggered = _motor->get_is_stalled();
    }

    _is_motor_monitor_stall_guard_triggered = _motor_monitor->is_motor_stalled();

    if (!_is_tmc_stall_guard_triggered && !_is_motor_monitor_stall_guard_triggered)
    {
      return false;
    }

    if (_is_tmc_stall_guard_triggered)
    {
      debug_println("[ElectricalDrawer]: Motor stall is detected by TMC stall guard.!");
    }
    if (_is_motor_monitor_stall_guard_triggered)
    {
      debug_println("[ElectricalDrawer]: Motor stall is detected by motor monitor.");
    }

    return true;
  }

  void ElectricalDrawer::handle_stall_guard_triggered()
  {
    debug_println("[ElectricalDrawer]: Stall guard is triggered! Setting speed to 0 and creating feedback messages!");

    _motor->set_target_speed_instantly(0);

    _motor->reset_stall_guard();

    _is_idling = true;

    _can_utils->enqueue_e_drawer_feedback_msg(
      _module_id,
      _id,
      _endstop_switch->is_switch_pressed(),
      _electrical_drawer_lock.has_value() ? _electrical_drawer_lock.value()->is_lock_switch_pushed() : false,
      MOTOR_IS_STALLED,
      _encoder->get_normed_current_position(),
      PUSH_TO_CLOSE_NOT_TRIGGERED);

    _timestamp_stall_guard_triggered_in_ms = millis();
  }

  void ElectricalDrawer::reset_encoder_if_endstop_is_pushed()
  {
    if (_endstop_switch->is_switch_pressed())
    {
      _encoder->set_current_position(0);
    }
  }

  void ElectricalDrawer::add_e_drawer_task_to_queue(const EDrawerTask &e_drawer_task)
  {
    // Before we add a new task to the queue, we need to check if the drawer was homed once
    if (!_drawer_was_homed_once)
    {
      debug_println("[ElectricalDrawer]: Drawer was not homed once yet, so add homing task to queue!");
      _e_drawer_task_queue->add_element_to_queue(
        {DRAWER_TARGET_HOMING_POSITION,
         get_normed_target_speed_uint8(_config->get_drawer_initial_homing_speed()),
         STALL_GUARD_DISABLED,
         IS_HOMING,
         false});
    }

    debug_printf(
      "[ElectricalDrawer]: Adding new e-drawer task to queue! Current position: %d, Target position: %d, Target "
      "speed: %d, Stall guard value: %d\n",
      _encoder->get_normed_current_position(),
      e_drawer_task.target_position,
      e_drawer_task.target_speed,
      e_drawer_task.stall_guard_value);

    _e_drawer_task_queue->add_element_to_queue(e_drawer_task);
  }

  uint32_t ElectricalDrawer::get_normed_target_speed_uint32(const uint8_t target_speed) const
  {
    uint32_t max_speed = _config->get_drawer_max_speed();
    uint32_t target_speed_casted = static_cast<uint32_t>(target_speed);

    return (target_speed_casted * max_speed) / UINT8_MAX;
  }

  uint8_t ElectricalDrawer::get_normed_target_speed_uint8(const uint32_t target_speed) const
  {
    return (target_speed * UINT8_MAX) / _config->get_drawer_max_speed();
  }

  void ElectricalDrawer::set_target_speed_and_direction(const uint8_t target_speed, const bool use_acceleration_ramp)
  {
    uint32_t normed_target_speed_uint32 = get_normed_target_speed_uint32(target_speed);
    _is_drawer_moving_out ? _motor->set_direction(stepper_motor::counter_clockwise)
                          : _motor->set_direction(stepper_motor::clockwise);

    if (use_acceleration_ramp)
    {
      _motor->set_target_speed_with_accelerating_ramp(normed_target_speed_uint32,
                                                      _config->get_drawer_default_acceleration());
    }
    else
    {
      _motor->set_target_speed_instantly(normed_target_speed_uint32);
    }

    _encoder->init_encoder_before_next_movement(_is_drawer_moving_out);
  }

  void ElectricalDrawer::handle_decelerating_for_moving_in_drawer()
  {
    _triggered_deceleration_for_drawer_moving_in = true;

    _motor->set_target_speed_with_decelerating_ramp(
      _config->get_drawer_homing_speed(),
      _encoder->convert_uint8_position_to_drawer_position_scale(_config->get_drawer_moving_in_deceleration_distance()),
      _encoder->get_current_position());
  }

  void ElectricalDrawer::handle_decelerating_for_moving_out_drawer()
  {
    if (!_triggered_deceleration_for_drawer_moving_out &&
        (_encoder->get_normed_current_position() + _config->get_drawer_moving_out_deceleration_distance()) >=
          _target_position_uint8)
    {
      debug_printf(
        "[ElectricalDrawer]: E-drawer is moving out and will now be decelerated! normed_current_position_uint8 = "
        "%d, "
        "_target_position_uint8 = %d\n",
        _encoder->get_normed_current_position(),
        _target_position_uint8);
      _triggered_deceleration_for_drawer_moving_out = true;
      _motor->set_target_speed_with_decelerating_ramp(0,
                                                      _encoder->convert_uint8_position_to_drawer_position_scale(
                                                        _config->get_drawer_moving_out_deceleration_distance()),
                                                      _encoder->get_current_position());
    }
  }

  void ElectricalDrawer::handle_finished_moving_in_drawer()
  {
    bool is_drawer_closed;
    if (_electrical_drawer_lock.has_value())
    {
      is_drawer_closed =
        _endstop_switch->is_switch_pressed() && !_electrical_drawer_lock.value()->is_lock_switch_pushed();
    }
    else
    {
      is_drawer_closed = _endstop_switch->is_switch_pressed();
    }

    if (is_drawer_closed)
    {
      handle_drawer_just_closed();
    }
  }

  void ElectricalDrawer::handle_finished_moving_out_drawer()
  {
    if (_encoder->get_normed_current_position() >= _target_position_uint8)
    {
      debug_printf(
        "[ElectricalDrawer]: Moving e-drawer out is finished! normed_current_position_uint8: %d, "
        "_target_position_uint8: %d\n",
        _encoder->get_normed_current_position(),
        _target_position_uint8);
      _motor->set_target_speed_instantly(0);
      _can_utils->enqueue_e_drawer_feedback_msg(_module_id,
                                                _id,
                                                _endstop_switch->is_switch_pressed(),
                                                false,
                                                is_stall_guard_triggered(),
                                                _encoder->get_normed_current_position(),
                                                PUSH_TO_CLOSE_NOT_TRIGGERED);
      _triggered_deceleration_for_drawer_moving_out = false;
      _is_idling = true;
      return;
    }
  }

  void ElectricalDrawer::handle_drawer_moving_in()
  {
    if (!_triggered_deceleration_for_drawer_moving_in &&
        _encoder->get_normed_current_position() < _config->get_drawer_moving_in_deceleration_distance())
    {
      handle_decelerating_for_moving_in_drawer();
    }
    else if (_encoder->get_normed_current_position() < _config->get_drawer_moving_in_final_homing_distance())
    {
      _motor->set_target_speed_instantly(_config->get_drawer_homing_speed());

      handle_finished_moving_in_drawer();
    }
  }

  void ElectricalDrawer::handle_drawer_moving_out()
  {
    handle_decelerating_for_moving_out_drawer();

    handle_finished_moving_out_drawer();
  }

  void ElectricalDrawer::handle_drawer_just_opened()
  {
    bool is_drawer_retracted = _endstop_switch->is_switch_pressed();

    if (_electrical_drawer_lock.value()->is_drawer_opening_in_progress() && !is_drawer_retracted &&
        !_triggered_closing_lock_after_opening)
    {
      _electrical_drawer_lock.value()->set_open_lock_current_step(
        false);   // this makes sure the lock automatically closes as soon as the drawer is opened
      _triggered_closing_lock_after_opening =
        true;   // this makes sure, closing the lock is only triggered once and not permanently
      debug_println("[ElectricalDrawer]: Triggered closing the lock because drawer is not retracted anymore!");
    }
  }

  void ElectricalDrawer::handle_drawer_just_closed()
  {
    if (_electrical_drawer_lock.has_value())
    {
      if (_electrical_drawer_lock.value()->is_drawer_opening_in_progress())
      {
        // reset these flags for the next opening of the drawer
        _electrical_drawer_lock.value()->set_drawer_opening_is_in_progress(false);
      }
      else
      {
        debug_println("[ElectricalDrawer]: Drawer is closed, but no opening was in progress!");
        return;
      }
    }

    debug_println("[ElectricalDrawer]: Drawer is closed! Setting speed to 0 and creating feedback messages!");
    _motor->set_target_speed_instantly(0);
    _encoder->set_current_position(0);

    _is_idling = true;
    _triggered_closing_lock_after_opening = false;
    _triggered_deceleration_for_drawer_moving_in = false;

    _can_utils->enqueue_e_drawer_feedback_msg(
      _module_id,
      _id,
      _endstop_switch->is_switch_pressed(),
      _electrical_drawer_lock.has_value() ? _electrical_drawer_lock.value()->is_lock_switch_pushed() : false,
      is_stall_guard_triggered(),
      _encoder->get_normed_current_position(),
      PUSH_TO_CLOSE_NOT_TRIGGERED);
    _can_utils->enqueue_drawer_feedback_msg(
      _module_id,
      _id,
      _endstop_switch->is_switch_pressed(),
      _electrical_drawer_lock.has_value() ? _electrical_drawer_lock.value()->is_lock_switch_pushed() : false);
  }

  bool ElectricalDrawer::is_stall_guard_triggered() const
  {
    return _is_motor_monitor_stall_guard_triggered || _is_tmc_stall_guard_triggered;
  }

  void ElectricalDrawer::debug_prints_moving_electrical_drawer()
  {
    int normed_target_position = (_target_position_uint8 / 255.0) * _encoder->get_count_drawer_max_extent();
    // debug_printf("[ElectricalDrawer]: Current position: % d, Target Position: %d, Current Speed: %d, Target
    // Speed: %d\n",
    //              _encoder->get_current_position(),
    //              normed_target_position,
    //              _motor->get_active_speed(),
    //              _motor->get_target_speed());
  }

}   // namespace drawer_controller