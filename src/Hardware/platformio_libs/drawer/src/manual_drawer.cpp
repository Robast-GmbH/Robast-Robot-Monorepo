#include "drawer/manual_drawer.hpp"

namespace drawer
{
  ManualDrawer::ManualDrawer(const uint32_t module_id,
                             const uint8_t id,
                             const std::shared_ptr<can_toolbox::CanUtils> can_utils,
                             const std::shared_ptr<switch_lib::Switch> endstop_switch,
                             const std::shared_ptr<lock::ElectricalDrawerLock> drawer_lock)
      : _module_id{module_id},
        _id{id},
        _can_utils{can_utils},
        _endstop_switch{endstop_switch},
        _drawer_lock{drawer_lock}
  {
  }

  void ManualDrawer::set_motor_driver_state(const bool enabled, const uint8_t motor_id) const
  {
    Serial.println(
      "[ManualDrawer]: Warning! Manual drawer does not support motor driver state control. This should not occur.");
    _can_utils->enqueue_error_feedback_msg(
      _module_id, _id, robast_can_msgs::can_data::error_code::MOTOR_DRIVER_STATE_CONTROL_NOT_SUPPORTED_BY_MODULE);
  }

  void ManualDrawer::update_state()
  {
    handle_drawer_lock_control();
    handle_drawer_just_opened();
    handle_drawer_just_closed();
    _endstop_switch->update_sensor_value();
  }

  void ManualDrawer::unlock()
  {
    _drawer_lock->unlock();
  }

  void ManualDrawer::handle_drawer_lock_control()
  {
    _drawer_lock->update_sensor_values();

    _drawer_lock->handle_lock_control();

    if (_drawer_lock->is_drawer_auto_close_timeout_triggered())
    {
      _can_utils->enqueue_error_feedback_msg(
        _module_id, _id, robast_can_msgs::can_data::error_code::TIMEOUT_DRAWER_NOT_OPENED);
      _drawer_lock->set_drawer_auto_close_timeout_triggered(false);
    }
  }

  void ManualDrawer::handle_drawer_just_opened()
  {
    bool is_drawer_open = !_endstop_switch->is_switch_pressed();

    if (_drawer_lock->is_drawer_opening_in_progress() && is_drawer_open && !_triggered_closing_lock_after_opening)
    {
      // This makes sure the lock automatically closes as soon as the drawer is opened.
      _drawer_lock->set_expected_lock_state_current_step(lock::LockState::locked);

      _triggered_closing_lock_after_opening = true;   // makes sure the feedback msg is only sent once
      _can_utils->enqueue_drawer_feedback_msg(
        _module_id, _id, _endstop_switch->is_switch_pressed(), _drawer_lock->is_lock_switch_pushed());
    }
  }

  void ManualDrawer::handle_drawer_just_closed()
  {
    bool is_drawer_closed = _endstop_switch->is_switch_pressed() && !_drawer_lock->is_lock_switch_pushed();

    if (_drawer_lock->is_drawer_opening_in_progress() && is_drawer_closed && _triggered_closing_lock_after_opening)
    {
      _drawer_lock->set_is_drawer_opening_in_progress(false);
      _triggered_closing_lock_after_opening = false;   // reset this flag for the next opening of the drawer
      _can_utils->enqueue_drawer_feedback_msg(
        _module_id, _id, _endstop_switch->is_switch_pressed(), _drawer_lock->is_lock_switch_pushed());
    }
  }

  void ManualDrawer::add_e_drawer_task_to_queue(const utils::EDrawerTask &e_drawer_task)
  {
    Serial.println("[ManualDrawer]: Warnign! Manual drawer does not support e drawer tasks. This should not occur.");
    _can_utils->enqueue_error_feedback_msg(
      _module_id, _id, robast_can_msgs::can_data::error_code::E_DRAWER_TASK_NOT_SUPPORTED_BY_MODULE);
  }

}   // namespace drawer