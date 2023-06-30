#include "electrical_drawer.hpp"

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
                                     uint8_t motor_driver_address)
      : _module_id{module_id},
        _id{id},
        _can_db{can_db},
        _gpio_wrapper{gpio_wrapper},
        _stepper_pin_id_config{stepper_pin_id_config},
        _use_encoder{use_encoder},
        _electrical_lock{std::make_unique<ElectricalLock>(gpio_wrapper)}
  {
    if (use_encoder)
    {
      _encoder = std::make_unique<ESP32Encoder>(true);
      ESP32Encoder::useInternalWeakPullResistors = UP;
      _encoder->attachFullQuad(encoder_pin_a, encoder_pin_b);
      _encoder->setCount(0);
    }

    _motor = std::make_unique<stepper_motor::Motor>(motor_driver_address, _gpio_wrapper, _stepper_pin_id_config);
    _feedback_msg_queue.clear();
    _head_of_feedback_msg_queue = 0;
  }

  void ElectricalDrawer::init_electrical_lock(uint8_t pwr_open_lock_pin_id,
                                              uint8_t pwr_close_lock_pin_id,
                                              uint8_t sensor_lock_pin_id,
                                              uint8_t sensor_drawer_closed_pin_id)
  {
    _electrical_lock->initialize_lock(
      pwr_open_lock_pin_id, pwr_close_lock_pin_id, sensor_lock_pin_id, sensor_drawer_closed_pin_id);
  }

  void ElectricalDrawer::stop_motor()
  {
    _motor->set_target_speed(0, 0);
  }

  void ElectricalDrawer::start_motor()
  {
    _motor->set_target_speed(1000);
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
      Serial.print("Received request to unlock the lock!");
      _electrical_lock->unlock(_id);
      debug_prints_drawer_lock(msg);
      _electrical_drawer_opening_in_progress = true;
    }
  }

  std::optional<robast_can_msgs::CanMessage> ElectricalDrawer::can_out()
  {
    return get_element_from_feedback_msg_queue();
  }

  void ElectricalDrawer::update_state()
  {
    if (!_electrical_drawer_opening_in_progress)
    {
      return;
    }

    _electrical_lock->handle_lock_control();
    _electrical_lock->handle_reading_sensors();

    _motor->handle_motor_control();

    if (_motor->get_active_speed() == 0)
    {
      return;
    }

    update_position();

    handle_drawer_just_opened();

    check_if_motion_is_finished();
  }

  int32_t ElectricalDrawer::get_integrated_drawer_position()
  {
    stepper_motor::Direction direction = _motor->get_direction();
    int32_t integrated_position = 0;
    uint32_t current_timestemp = millis();

    integrated_position =
      (current_timestemp - _last_timestemp) * _motor->get_active_speed() / DRAWER_POSITION_OPEN_LOOP_INTEGRAL_GAIN;

    if (!_is_drawer_moving_out)
    {
      integrated_position *= -1;
    }
    _last_timestemp = current_timestemp;

    return integrated_position;
  }

  void ElectricalDrawer::unlock()
  {
    if (_electrical_lock->is_drawer_opening_in_progress())
    {
      Serial.printf("Drawer%d opening is already in progress, so lock won't be opened again!\n", _id);
    }
    else
    {
      _electrical_lock->set_open_lock_current_step(true);
      _electrical_lock->set_timestamp_last_lock_change();
      _electrical_lock->set_drawer_opening_is_in_progress(true);
    }
  }

  void ElectricalDrawer::check_if_drawer_is_homed()
  {
    if (_electrical_lock->is_endstop_switch_pushed())
    {
      _current_position_int32 = 0;
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

    if (_target_position_uint8 == _current_position_int32)
    {
      create_electrical_drawer_feedback_msg();
      return;
    }

    set_target_speed_and_direction(target_speed);
  }

  void ElectricalDrawer::set_target_speed_and_direction(uint8_t target_speed)
  {
    bool is_drawer_retracted = _electrical_lock->is_endstop_switch_pushed();
    bool is_lock_open = _electrical_lock->is_lock_switch_pushed();

    if ((is_drawer_retracted && is_lock_open) || (!is_drawer_retracted))
    {
      uint32_t normed_target_speed = (target_speed * DRAWER_MAX_SPEED) / UINT8_MAX;
      _is_drawer_moving_out = _target_position_uint8 > _current_position_int32;
      _is_drawer_moving_out ? _motor->set_direction(stepper_motor::counter_clockwise)
                            : _motor->set_direction(stepper_motor::clockwise);

      _motor->set_target_speed(normed_target_speed);

      if (!_use_encoder)
      {
        _last_timestemp = millis();
      }
    }
    else
    {
      Serial.println(
        "The electrical drawer can't be moved because the lock is not opened or the drawer is already retracted!");
    }
  }

  void ElectricalDrawer::create_electrical_drawer_feedback_msg()
  {
    robast_can_msgs::CanMessage can_msg_electrical_drawer_feedback =
      _can_db->can_messages.at(CAN_MSG_ELECTRICAL_DRAWER_FEEDBACK);
    std::vector can_signals_electrical_drawer_feedback = can_msg_electrical_drawer_feedback.get_can_signals();

    can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_MODULE_ID).set_data(_module_id);
    can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_DRAWER_ID).set_data(_id);

    const bool is_endstop_switch_pushed = _electrical_lock->is_endstop_switch_pushed();
    can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED).set_data(is_endstop_switch_pushed);

    const bool is_lock_switch_pushed = _electrical_lock->is_lock_switch_pushed();
    can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED).set_data(is_lock_switch_pushed);

    const bool is_drawer_stall_guard_triggered = _motor->get_is_stalled();
    can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_DRAWER_IS_STALL_GUARD_TRIGGERED)
      .set_data(is_lock_switch_pushed);

    uint8_t normed_current_position = get_normed_current_position();
    can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_DRAWER_POSITION).set_data(normed_current_position);

    can_msg_electrical_drawer_feedback.set_can_signals(can_signals_electrical_drawer_feedback);

    Serial.printf("Creating feedback_msg for module_id %d, drawer_id %d and position: %d\n",
                  _module_id,
                  _id,
                  normed_current_position);

    add_element_to_feedback_msg_queue(can_msg_electrical_drawer_feedback);
  }

  void ElectricalDrawer::update_position()
  {
    if (_use_encoder)
    {
      _current_position_int32 = _encoder->getCount();
    }
    else
    {
      _current_position_int32 += get_integrated_drawer_position();
      if (_current_position_int32 < 0)
      {
        _current_position_int32 = 0;
      }
    }
  }

  void ElectricalDrawer::check_if_motion_is_finished()
  {
    uint8_t normed_current_position_uint8 = get_normed_current_position();

    if (_is_drawer_moving_out && (normed_current_position_uint8 >= _target_position_uint8))
    {
      Serial.printf("Moving e-drawer out is finished! normed_current_position_uint8: %d, _target_position_uint8: %d\n",
                    normed_current_position_uint8,
                    _target_position_uint8);
      _motor->set_target_speed(0, 0);   // TODO@Jacob: Do not set the target_speed in dt = 0
      create_electrical_drawer_feedback_msg();
      return;
    }

    if (!_is_drawer_moving_out)
    {
      handle_drawer_moving_in(normed_current_position_uint8);
    }
  }

  void ElectricalDrawer::handle_drawer_moving_in(uint8_t normed_current_position_uint8)
  {
    if (normed_current_position_uint8 < DRAWER_HOMING_EXTENT)
    {
      if (!_homing_initialized)
      {
        initialize_homing();
      }

      bool is_drawer_closed =
        _electrical_lock->is_endstop_switch_pushed() && !_electrical_lock->is_lock_switch_pushed();
      if (is_drawer_closed)
      {
        handle_drawer_just_closed();
      }
    }
  }

  void ElectricalDrawer::initialize_homing()
  {
    _homing_initialized = true;
    float active_speed = _motor->get_active_speed();
    float percentage_of_max_speed = active_speed / DRAWER_MAX_SPEED;
    float homing_percentage_of_max_extent = ((float) DRAWER_HOMING_EXTENT) / 255.0;
    uint16_t time_to_slow_down_in_us = (100 / percentage_of_max_speed) / homing_percentage_of_max_extent;
    Serial.printf(
      "electrical_drawer.cpp, handle_drawer_moving_in(): Starting homing with time_to_slow_down_in_us = %d, "
      "percentage_of_max_speed = %f, "
      "homing_percentage_of_max_extent = %f\n",
      time_to_slow_down_in_us,
      percentage_of_max_speed,
      homing_percentage_of_max_extent);
    _motor->set_target_speed(DRAWER_HOMING_SPEED, time_to_slow_down_in_us);
  }

  uint8_t ElectricalDrawer::get_normed_current_position()
  {
    uint32_t normed_current_position_uint32 = (_current_position_int32 * 255) / DRAWER_MAX_EXTENT;

    if (normed_current_position_uint32 > 255)
    {
      return 255;
    }
    else
    {
      return normed_current_position_uint32;
    }
  }

  void ElectricalDrawer::handle_drawer_just_opened()
  {
    bool is_drawer_retracted = _electrical_lock->is_endstop_switch_pushed();
    if (_electrical_lock->is_drawer_opening_in_progress() && !is_drawer_retracted &&
        !_triggered_closing_lock_after_opening)
    {
      _electrical_lock->set_open_lock_current_step(
        false);   // this makes sure the lock automatically closes as soon as the drawer is opened
      _triggered_closing_lock_after_opening =
        true;     // this makes sure, closing the lock is only triggered once and not permanently
      Serial.println("Triggered closing the lock because drawer is not retracted anymore!");
    }
  }

  void ElectricalDrawer::handle_drawer_just_closed()
  {
    if (_electrical_lock->is_drawer_opening_in_progress())
    {
      Serial.println("Drawer is closed! Setting speed to 0 and creating feedback messages!");
      _motor->set_target_speed(0, 0);
      if (_use_encoder)
      {
        _encoder->setCount(0);
      }
      _current_position_int32 = 0;

      // reset these flags for the next opening of the drawer
      _electrical_lock->set_drawer_opening_is_in_progress(false);
      _electrical_drawer_opening_in_progress = false;
      _triggered_closing_lock_after_opening = false;
      _homing_initialized = false;
      create_electrical_drawer_feedback_msg();
      create_drawer_closed_feedback_can_msg();
    }
  }

  void ElectricalDrawer::create_drawer_closed_feedback_can_msg()
  {
    robast_can_msgs::CanMessage can_msg_drawer_feedback = this->_can_db->can_messages.at(CAN_MSG_DRAWER_FEEDBACK);
    std::vector can_signals_drawer_feedback = can_msg_drawer_feedback.get_can_signals();

    can_signals_drawer_feedback.at(CAN_SIGNAL_MODULE_ID).set_data(_module_id);
    can_signals_drawer_feedback.at(CAN_SIGNAL_DRAWER_ID).set_data(_id);

    const bool is_endstop_switch_pushed = _electrical_lock->is_endstop_switch_pushed();
    can_signals_drawer_feedback.at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED).set_data(is_endstop_switch_pushed);

    const bool is_lock_switch_pushed = _electrical_lock->is_lock_switch_pushed();
    can_signals_drawer_feedback.at(CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED).set_data(is_lock_switch_pushed);

    can_msg_drawer_feedback.set_can_signals(can_signals_drawer_feedback);

    add_element_to_feedback_msg_queue(can_msg_drawer_feedback);
  }

  void ElectricalDrawer::add_element_to_feedback_msg_queue(robast_can_msgs::CanMessage feedback_msg)
  {
    _feedback_msg_queue.push_back(feedback_msg);
  }

  std::optional<robast_can_msgs::CanMessage> ElectricalDrawer::get_element_from_feedback_msg_queue()
  {
    uint8_t num_of_msgs_in_queue = _feedback_msg_queue.size();
    if (num_of_msgs_in_queue == 0)
    {
      return {};
    }

    if (_head_of_feedback_msg_queue == (num_of_msgs_in_queue - 1))
    {
      robast_can_msgs::CanMessage feedback_can_msg = _feedback_msg_queue[_head_of_feedback_msg_queue];
      _feedback_msg_queue.clear();
      _head_of_feedback_msg_queue = 0;
      return feedback_can_msg;
    }
    else
    {
      return _feedback_msg_queue[_head_of_feedback_msg_queue++];
    }
  }

  void ElectricalDrawer::debug_prints_moving_electrical_drawer()
  {
    int normed_target_position = (_target_position_uint8 / 255.0) * DRAWER_MAX_EXTENT;
    Serial.printf(
      "Current position: % d, Target Position: %d, Current Speed: %d, Target Speed: %d, _last_timestemp: %d, "
      "millis(): %d\n",
      _current_position_int32,
      normed_target_position,
      _motor->get_active_speed(),
      _motor->get_target_speed(),
      _last_timestemp,
      millis());
  }

  void ElectricalDrawer::debug_prints_electric_drawer_task(robast_can_msgs::CanMessage can_message)
  {
    Serial.print("Standard ID: ");
    Serial.print(can_message.get_id(), HEX);
    Serial.print(" rx_dlc: ");
    Serial.print(can_message.get_dlc(), DEC);
    Serial.print(" MODULE ID: ");
    Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_MODULE_ID).get_data(), HEX);
    Serial.print(" DRAWER ID: ");
    Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_ID).get_data(), HEX);
    Serial.print(" GOTO POSITION: ");
    Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_TARGET_POSITION).get_data(), DEC);
    Serial.print(" SPEED: ");
    Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_SPEED).get_data(), DEC);
    Serial.print(" STALL GUARD ENABLE: ");
    Serial.println(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_STALL_GUARD_ENABLE).get_data(), DEC);
  }

  void ElectricalDrawer::debug_prints_drawer_lock(robast_can_msgs::CanMessage& can_message)
  {
    Serial.print("Standard ID: ");
    Serial.print(can_message.get_id(), HEX);
    Serial.print(" rx_dlc: ");
    Serial.print(can_message.get_dlc(), DEC);
    Serial.print(" MODULE ID: ");
    Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_MODULE_ID).get_data(), HEX);
    Serial.print(" DRAWER ID: ");
    Serial.println(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_ID).get_data(), HEX);
  }

}   // namespace drawer_controller