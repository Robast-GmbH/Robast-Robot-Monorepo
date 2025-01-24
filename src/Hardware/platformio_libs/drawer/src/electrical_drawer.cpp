#include "drawer/electrical_drawer.hpp"

namespace drawer
{
  ElectricalDrawer::ElectricalDrawer(const uint32_t module_id,
                                     const uint8_t id,
                                     const std::shared_ptr<can_toolbox::CanUtils> can_utils,
                                     const std::shared_ptr<switch_lib::Switch> endstop_switch,
                                     const std::shared_ptr<ElectricalDrawerConfig> e_drawer_config,
                                     const std::shared_ptr<drawer::MotionController> motion_controller,
                                     const std::optional<std::shared_ptr<lock::ElectricalDrawerLock>> drawer_lock)
      : _module_id{module_id},
        _id{id},
        _can_utils{can_utils},
        _endstop_switch{endstop_switch},
        _drawer_lock{drawer_lock},
        _config{e_drawer_config},
        _motion_controller{motion_controller}
  {
    init();
  }

  void ElectricalDrawer::unlock()
  {
    if (_drawer_lock.has_value())
    {
      debug_println("[ElectricalDrawer]: Received request to unlock the lock!");
      _drawer_lock.value()->unlock();
    }
    else
    {
      serial_println_warning(
        "[ElectricalDrawer]: Warning! Received request to unlock the lock, but no lock is installed!");
    }
  }

  void ElectricalDrawer::update_state()
  {
    if (_drawer_lock.has_value())
    {
      handle_drawer_lock_control();
    }

    // updating the sensor values needs to be done all the time because of the moving average calculation
    _endstop_switch->update_sensor_value();

    if (_motion_controller->is_idling())
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
    if (_endstop_switch->is_switch_pressed())
    {
      _motion_controller->reset_encoder();
    }

    if (_motion_controller->is_push_to_close_is_triggered())
    {
      handle_push_to_close_triggered();
    }

    check_if_drawer_is_pulled_out();

    // In very rare cases, when the stall guard is triggerd very close before closing, we might close the drawer
    // by hand without push to close triggering. In this case, we need to catch the drawer closing.
    if (_motion_controller->is_drawer_opening_in_progress() && is_drawer_closed())
    {
      handle_drawer_just_closed();
    }

    start_next_e_drawer_task();
  }

  void ElectricalDrawer::check_if_drawer_is_pulled_out()
  {
    if (_motion_controller->is_drawer_pulled_out())
    {
      debug_printf_warning("[ElectricalDrawer]: Drawer is pulled out! Adding task to move drawer back in!\n");
      add_e_drawer_task_to_queue({DRAWER_TARGET_HOMING_POSITION,
                                  _config->get_drawer_push_in_auto_close_speed(),
                                  _config->get_drawer_push_in_auto_close_stall_guard_value(),
                                  IS_NOT_HOMING,
                                  DO_NOT_USE_ACCELERATION_RAMP});
      // TODO: Do we need to send a feedback message here?
    }
  }

  void ElectricalDrawer::start_next_e_drawer_task()
  {
    const std::optional<utils::EDrawerTask> e_drawer_task = _e_drawer_task_queue->dequeue();
    if (e_drawer_task.has_value())
    {
      debug_printf("[ElectricalDrawer]: Received new e-drawer task from queue with target position %u and speed %u!\n",
                   e_drawer_task.value().target_position,
                   e_drawer_task.value().target_speed);

      _motion_controller->start_e_drawer_task(e_drawer_task.value());

      if (!e_drawer_task.value().is_homing)
      {
        if (e_drawer_task.value().target_position == _motion_controller->get_normed_current_position())
        {
          _can_utils->enqueue_e_drawer_feedback_msg(_module_id,
                                                    _id,
                                                    _endstop_switch->is_switch_pressed(),
                                                    LOCK_SWITCH_IS_NOT_PUSHED,
                                                    _motion_controller->is_stall_guard_triggered(),
                                                    _motion_controller->get_normed_current_position(),
                                                    PUSH_TO_CLOSE_NOT_TRIGGERED);
        }
        else
        {
          // We need to reset the stall guard before we start a new movement because stall guard is a status at the
          // moment
          // TODO: "stall guard triggerd" should be an event not a status
          _can_utils->enqueue_e_drawer_feedback_msg(
            _module_id,
            _id,
            _endstop_switch->is_switch_pressed(),
            _drawer_lock.has_value() ? _drawer_lock.value()->is_lock_switch_pushed() : false,
            MOTOR_IS_NOT_STALLED,
            _motion_controller->get_normed_current_position(),
            PUSH_TO_CLOSE_NOT_TRIGGERED);
        }
      }
    }
  }

  void ElectricalDrawer::handle_drawer_active_state()
  {
    if (!_motion_controller->was_drawer_homed_once())
    {
      if (_endstop_switch->is_switch_pressed())
      {
        _motion_controller->handle_finished_drawer_homing();
      }
      return;
    }

    if (_motion_controller->is_stall_guard_triggered())
    {
      _motion_controller->handle_stall_guard_triggered();

      check_for_drawer_not_opened_error();
      return;
    }

    _motion_controller->handle_motor_control();

    _motion_controller->update_position();

    _motion_controller->is_drawer_moving_out() ? handle_drawer_moving_out() : handle_drawer_moving_in();
  }

  void ElectricalDrawer::handle_drawer_lock_control()
  {
    _drawer_lock.value()->update_sensor_values();

    _drawer_lock.value()->handle_lock_control();

    if (_drawer_lock.value()->is_drawer_auto_close_timeout_triggered())
    {
      _can_utils->enqueue_error_feedback_msg(
        _module_id, _id, robast_can_msgs::can_data::error_code::TIMEOUT_DRAWER_NOT_OPENED);
      _drawer_lock.value()->set_drawer_auto_close_timeout_triggered(false);
    }

    handle_drawer_just_opened();
  }

  void ElectricalDrawer::add_e_drawer_task_to_queue(const utils::EDrawerTask &e_drawer_task)
  {
    // Before we add a new task to the queue, we need to check if the drawer was homed once
    if (!_motion_controller->was_drawer_homed_once())
    {
      debug_printf_warning("[ElectricalDrawer]: Drawer was not homed once yet, so add homing task to queue!\n");
      _e_drawer_task_queue->enqueue(
        {DRAWER_TARGET_HOMING_POSITION,
         _motion_controller->get_normed_target_speed_uint8(_config->get_drawer_initial_homing_speed()),
         STALL_GUARD_DISABLED,
         IS_HOMING,
         false});
    }

    debug_printf(
      "[ElectricalDrawer]: Adding new e-drawer task to queue! Target position: %d, Target "
      "speed: %d, Stall guard value: %d\n",
      e_drawer_task.target_position,
      e_drawer_task.target_speed,
      e_drawer_task.stall_guard_value);

    if (_motion_controller->is_task_redundant(e_drawer_task.target_position))
    {
      serial_printf_warning(
        "[ElectricalDrawer]: Warning! Received redundant task with target position %d! Discarding it!\n",
        e_drawer_task.target_position);
      return;
    }

    _e_drawer_task_queue->enqueue(e_drawer_task);
  }

  void ElectricalDrawer::handle_drawer_moving_in()
  {
    _motion_controller->handle_decelerating_for_moving_in_drawer();

    if (is_drawer_closed())
    {
      handle_drawer_just_closed();
    }
  }

  void ElectricalDrawer::handle_drawer_moving_out()
  {
    _motion_controller->handle_decelerating_for_moving_out_drawer();

    if (_motion_controller->handle_finished_moving_out_drawer())
    {
      _can_utils->enqueue_e_drawer_feedback_msg(
        _module_id,
        _id,
        _endstop_switch->is_switch_pressed(),
        _drawer_lock.has_value() ? _drawer_lock.value()->is_lock_switch_pushed() : false,
        _motion_controller->is_stall_guard_triggered(),
        _motion_controller->get_normed_current_position(),
        PUSH_TO_CLOSE_NOT_TRIGGERED);
    };
  }

  void ElectricalDrawer::handle_drawer_just_opened()
  {
    const bool is_drawer_retracted = _endstop_switch->is_switch_pressed();

    static bool is_lock_state_set = true;

    if (_drawer_lock.value()->is_drawer_opening_in_progress() && !is_drawer_retracted &&
        !_triggered_closing_lock_after_opening)
    {
      debug_printf_green(
        "[ElectricalDrawer, handle_drawer_just_opened]: Setting _triggered_closing_lock_after_opening to true!\n");

      // We want to wait a small moment before closing the lock to make sure the drawer had enough time to open
      _timestamp_drawer_opened_in_ms = millis();

      // This makes sure, closing the lock is only triggered once and not permanently.
      _triggered_closing_lock_after_opening = true;

      is_lock_state_set = false;
    }

    if (!is_lock_state_set &&
        millis() - _timestamp_drawer_opened_in_ms > _config->get_wait_time_to_close_lock_after_drawer_opened_in_ms())
    {
      // This makes sure the lock automatically closes as soon as the drawer is opened.
      _drawer_lock.value()->set_expected_lock_state_current_step(lock::LockState::locked);
      is_lock_state_set = true;
      debug_printf_green("[ElectricalDrawer]: Triggered closing the lock because drawer is not retracted anymore!\n");
    }
  }

  void ElectricalDrawer::handle_push_to_close_triggered()
  {
    debug_printf_green("[ElectricalDrawer]: Drawer is pushed in! Adding task to move drawer in!\n");
    add_e_drawer_task_to_queue({DRAWER_TARGET_HOMING_POSITION,
                                _config->get_drawer_push_in_auto_close_speed(),
                                _config->get_drawer_push_in_auto_close_stall_guard_value(),
                                IS_NOT_HOMING,
                                DO_NOT_USE_ACCELERATION_RAMP});
    _can_utils->enqueue_e_drawer_feedback_msg(
      _module_id,
      _id,
      _endstop_switch->is_switch_pressed(),
      _drawer_lock.has_value() ? _drawer_lock.value()->is_lock_switch_pushed() : false,
      MOTOR_IS_NOT_STALLED,
      _motion_controller->get_normed_current_position(),
      PUSH_TO_CLOSE_TRIGGERED);
  }

  void ElectricalDrawer::handle_drawer_just_closed()
  {
    _motion_controller->handle_finished_moving_in_drawer();

    if (_drawer_lock.has_value())
    {
      if (_drawer_lock.value()->is_drawer_opening_in_progress())
      {
        _drawer_lock.value()->set_is_drawer_opening_in_progress(false);   // reset flag for next opening of drawer
      }
      else
      {
        debug_printf_warning("[ElectricalDrawer]: Drawer is closed, but no opening was in progress!\n");
        return;
      }
    }
    _triggered_closing_lock_after_opening = false;

    _can_utils->enqueue_e_drawer_feedback_msg(_module_id,
                                              _id,
                                              ENDSTOP_SWITCH_IS_PUSHED,
                                              LOCK_SWITCH_IS_NOT_PUSHED,
                                              _motion_controller->is_stall_guard_triggered(),
                                              _motion_controller->get_normed_current_position(),
                                              PUSH_TO_CLOSE_NOT_TRIGGERED);

    if (_motion_controller->is_idling())
    {
      // if the drawer was closed within the idle state, we need to send an error message because this is not intended
      _can_utils->enqueue_error_feedback_msg(
        _module_id, _id, robast_can_msgs::can_data::error_code::DRAWER_CLOSED_IN_IDLE_STATE);
    }
    else
    {
      _motion_controller->set_is_idling(true);
    }
  }

  bool ElectricalDrawer::is_drawer_closed() const
  {
    if (_drawer_lock.has_value())
    {
      return _endstop_switch->is_switch_pressed() && !_drawer_lock.value()->is_lock_switch_pushed();
    }
    else
    {
      return _endstop_switch->is_switch_pressed();
    }
  }

  void ElectricalDrawer::check_for_drawer_not_opened_error() const
  {
    // If the drawer can't be moved close to it's home position, it could be that the drawer is stuck and could not
    // pass the lock, so we need to close the lock and sent an error message.
    if (_motion_controller->get_normed_current_position() <=
        _config->get_encoder_threshold_for_drawer_not_opened_during_stall())
    {
      _can_utils->enqueue_error_feedback_msg(
        _module_id, _id, robast_can_msgs::can_data::error_code::TIMEOUT_DRAWER_NOT_OPENED);
      if (_drawer_lock.has_value())
      {
        _drawer_lock.value()->set_expected_lock_state_current_step(lock::LockState::locked);
      }
    }
    else
    {
      _can_utils->enqueue_e_drawer_feedback_msg(
        _module_id,
        _id,
        _endstop_switch->is_switch_pressed(),
        _drawer_lock.has_value() ? _drawer_lock.value()->is_lock_switch_pushed() : false,
        MOTOR_IS_STALLED,
        _motion_controller->get_normed_current_position(),
        PUSH_TO_CLOSE_NOT_TRIGGERED);
    }
  }
}   // namespace drawer
