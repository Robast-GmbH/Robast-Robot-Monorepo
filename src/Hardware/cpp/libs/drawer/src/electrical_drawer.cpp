#include "drawer/electrical_drawer.hpp"

namespace drawer_controller
{
  ElectricalDrawer::ElectricalDrawer(uint32_t module_id,
                                     uint8_t id,
                                     std::shared_ptr<robast_can_msgs::CanDb> can_db,
                                     std::shared_ptr<IGpioWrapper> gpio_wrapper,
                                     const stepper_motor::StepperPinIdConfig& stepper_pin_id_config,
                                     bool use_encoder,
                                     uint8_t encoder_pin_a,
                                     uint8_t encoder_pin_b,
                                     uint8_t motor_driver_address,
                                     std::shared_ptr<Switch> endstop_switch,
                                     std::optional<std::shared_ptr<ElectricalDrawerLock>> electrical_drawer_lock)
      : _module_id{module_id},
        _id{id},
        _gpio_wrapper{gpio_wrapper},
        _stepper_pin_id_config{stepper_pin_id_config},
        _encoder{std::make_unique<Encoder>(use_encoder, encoder_pin_a, encoder_pin_b)},
        _can_utils{std::make_unique<CanUtils>(can_db)},
        _motor{std::make_unique<stepper_motor::Motor>(motor_driver_address, _gpio_wrapper, _stepper_pin_id_config)},
        _endstop_switch{endstop_switch},
        _electrical_drawer_lock{electrical_drawer_lock}
  {
  }

  void ElectricalDrawer::init()
  {
    if (_electrical_drawer_lock.has_value())
    {
      _electrical_drawer_lock.value()->initialize_lock();
    }
    init_motor();
  }

  void ElectricalDrawer::stop_motor()
  {
    _motor->set_target_speed_instantly(0);
  }

  void ElectricalDrawer::start_motor()
  {
    _motor->set_target_speed_instantly(1000);
  }

  void ElectricalDrawer::init_motor()
  {
    _motor->init();
  }

  void ElectricalDrawer::can_in(robast_can_msgs::CanMessage msg)
  {
    if (msg.get_id() == CAN_ID_ELECTRICAL_DRAWER_TASK)
    {
      handle_electrical_drawer_task_msg(msg);
      debug_prints_electric_drawer_task(msg);
    }
    if (msg.get_id() == CAN_ID_DRAWER_UNLOCK)
    {
      if (_electrical_drawer_lock.has_value())
      {
        debug_println("Received request to unlock the lock!");
        _electrical_drawer_lock.value()->unlock(_id);
        debug_prints_drawer_lock(msg);
        _is_idling = false;
      }
      else
      {
        Serial.println("Warning! Received request to unlock the lock, but no lock is installed!");
      }
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
  }

  void ElectricalDrawer::handle_drawer_active_state()
  {
    if (_electrical_drawer_lock.has_value())
    {
      handle_electrical_drawer_lock_control();
    }

    _motor->handle_motor_control(_encoder->get_current_position());

    if (_motor->get_active_speed() == 0)
    {
      return;
    }

    _encoder->update_position(_motor->get_active_speed());

    _is_drawer_moving_out ? handle_drawer_moving_out() : handle_drawer_moving_in();

    debug_prints_moving_electrical_drawer();
  }

  float ElectricalDrawer::get_moving_average_drawer_closed_pin()
  {
    return _moving_average_drawer_closed_pin;
  }

  void ElectricalDrawer::handle_electrical_drawer_lock_control()
  {
    _electrical_drawer_lock.value()->update_sensor_values();

    _electrical_drawer_lock.value()->handle_lock_control();

    if (_electrical_drawer_lock.value()->is_drawer_auto_close_timeout_triggered())
    {
      _can_utils->handle_error_feedback_msg(_module_id, _id, CAN_DATA_ERROR_CODE_TIMEOUT_DRAWER_NOT_OPENED);
      _electrical_drawer_lock.value()->set_drawer_auto_close_timeout_triggered(false);
    }

    handle_drawer_just_opened();
  }

  void ElectricalDrawer::check_if_drawer_is_homed()
  {
    if (_endstop_switch->is_switch_pressed())
    {
      _encoder->set_current_position(0);
    }
  }

  void ElectricalDrawer::handle_electrical_drawer_task_msg(robast_can_msgs::CanMessage can_message)
  {
    _target_position_uint8 = can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_TARGET_POSITION).get_data();
    uint8_t target_speed = can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_SPEED).get_data();
    _stall_guard_enabled = can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_STALL_GUARD_ENABLE).get_data() ==
                               CAN_DATA_ELECTRICAL_DRAWER_STALL_GUARD_ENABLED
                             ? true
                             : false;

    // motor_->setStallGuard(stall_guard_enabled_);

    check_if_drawer_is_homed();

    if (_target_position_uint8 == _encoder->get_current_position())
    {
      _can_utils->handle_electrical_drawer_feedback_msg(_module_id,
                                                        _id,
                                                        _endstop_switch->is_switch_pressed(),
                                                        false,
                                                        _motor->get_is_stalled(),
                                                        _encoder->get_normed_current_position());
      return;
    }

    set_target_speed_and_direction(target_speed);
  }

  void ElectricalDrawer::set_target_speed_and_direction(uint8_t target_speed)
  {
    uint32_t normed_target_speed_uint32 = (target_speed * DRAWER_MAX_SPEED) / UINT8_MAX;
    _is_drawer_moving_out = _target_position_uint8 > _encoder->get_normed_current_position();
    _is_drawer_moving_out ? _motor->set_direction(stepper_motor::counter_clockwise)
                          : _motor->set_direction(stepper_motor::clockwise);

    _motor->set_target_speed_with_accelerating_ramp(normed_target_speed_uint32, DEFAULT_DRAWER_ACCELERATION);

    _encoder->init_encoder_before_next_movement(_is_drawer_moving_out);
  }

  void ElectricalDrawer::handle_decelerating_for_moving_in_drawer()
  {
    _triggered_deceleration_for_drawer_moving_in = true;

    _motor->set_target_speed_with_decelerating_ramp(
      DRAWER_HOMING_SPEED,
      _encoder->convert_uint8_position_to_drawer_position_scale(DRAWER_MOVING_IN_DECELERATION_DISTANCE),
      _encoder->get_current_position());
  }

  void ElectricalDrawer::handle_decelerating_for_moving_out_drawer()
  {
    if (!_triggered_deceleration_for_drawer_moving_out &&
        (_encoder->get_normed_current_position() + DRAWER_MOVING_OUT_DECELERATION_DISTANCE) >= _target_position_uint8)
    {
      debug_printf(
        "E-drawer is moving out and will now be decelerated! normed_current_position_uint8 = %d, "
        "_target_position_uint8 = %d\n",
        _encoder->get_normed_current_position(),
        _target_position_uint8);
      _triggered_deceleration_for_drawer_moving_out = true;
      _motor->set_target_speed_with_decelerating_ramp(
        0,
        _encoder->convert_uint8_position_to_drawer_position_scale(DRAWER_MOVING_OUT_DECELERATION_DISTANCE),
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
      debug_printf("Moving e-drawer out is finished! normed_current_position_uint8: %d, _target_position_uint8: %d\n",
                   _encoder->get_normed_current_position(),
                   _target_position_uint8);
      _motor->set_target_speed_instantly(0);
      _can_utils->handle_electrical_drawer_feedback_msg(_module_id,
                                                        _id,
                                                        _endstop_switch->is_switch_pressed(),
                                                        false,
                                                        _motor->get_is_stalled(),
                                                        _encoder->get_normed_current_position());
      _triggered_deceleration_for_drawer_moving_out = false;
      return;
    }
  }

  void ElectricalDrawer::handle_drawer_moving_in()
  {
    if (!_triggered_deceleration_for_drawer_moving_in &&
        _encoder->get_normed_current_position() < DRAWER_MOVING_IN_DECELERATION_DISTANCE)
    {
      handle_decelerating_for_moving_in_drawer();
    }
    else if (_encoder->get_normed_current_position() < DRAWER_MOVING_IN_FINAL_HOMING_DISTANCE)
    {
      _motor->set_target_speed_instantly(DRAWER_HOMING_SPEED);

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
      debug_println("Triggered closing the lock because drawer is not retracted anymore!");
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
        debug_println("Drawer is closed, but no opening was in progress!");
        return;
      }
    }

    debug_println("Drawer is closed! Setting speed to 0 and creating feedback messages!");
    _motor->set_target_speed_instantly(0);
    _encoder->set_current_position(0);

    _is_idling = true;
    _triggered_closing_lock_after_opening = false;
    _triggered_deceleration_for_drawer_moving_in = false;

    _can_utils->handle_electrical_drawer_feedback_msg(
      _module_id,
      _id,
      _endstop_switch->is_switch_pressed(),
      _electrical_drawer_lock.has_value() ? _electrical_drawer_lock.value()->is_lock_switch_pushed() : false,
      _motor->get_is_stalled(),
      _encoder->get_normed_current_position());
    _can_utils->handle_drawer_feedback_msg(
      _module_id,
      _id,
      _endstop_switch->is_switch_pressed(),
      _electrical_drawer_lock.has_value() ? _electrical_drawer_lock.value()->is_lock_switch_pushed() : false);
  }

  void ElectricalDrawer::debug_prints_moving_electrical_drawer()
  {
    int normed_target_position = (_target_position_uint8 / 255.0) * DRAWER_MAX_EXTENT;
    debug_printf("Current position: % d, Target Position: %d, Current Speed: %d, Target Speed: %d\n",
                 _encoder->get_current_position(),
                 normed_target_position,
                 _motor->get_active_speed(),
                 _motor->get_target_speed());
  }

  void ElectricalDrawer::debug_prints_electric_drawer_task(robast_can_msgs::CanMessage can_message)
  {
    debug_print("Standard ID: ");
    debug_print_with_base(can_message.get_id(), HEX);
    debug_print(" rx_dlc: ");
    debug_print_with_base(can_message.get_dlc(), DEC);
    debug_print(" MODULE ID: ");
    debug_print_with_base(can_message.get_can_signals().at(CAN_SIGNAL_MODULE_ID).get_data(), HEX);
    debug_print(" DRAWER ID: ");
    debug_print_with_base(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_ID).get_data(), HEX);
    debug_print(" GOTO POSITION: ");
    debug_print_with_base(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_TARGET_POSITION).get_data(), DEC);
    debug_print(" SPEED: ");
    debug_print_with_base(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_SPEED).get_data(), DEC);
    debug_print(" STALL GUARD ENABLE: ");
    debug_println_with_base(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_STALL_GUARD_ENABLE).get_data(), DEC);
  }

  void ElectricalDrawer::debug_prints_drawer_lock(robast_can_msgs::CanMessage& can_message)
  {
    debug_print("Standard ID: ");
    debug_print_with_base(can_message.get_id(), HEX);
    debug_print(" rx_dlc: ");
    debug_print_with_base(can_message.get_dlc(), DEC);
    debug_print(" MODULE ID: ");
    debug_print_with_base(can_message.get_can_signals().at(CAN_SIGNAL_MODULE_ID).get_data(), HEX);
    debug_print(" DRAWER ID: ");
    debug_println_with_base(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_ID).get_data(), HEX);
  }

}   // namespace drawer_controller