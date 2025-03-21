#include "tray/electrical_tray_lock.hpp"

namespace tray
{
  ElectricalTrayLock::ElectricalTrayLock(const std::shared_ptr<interfaces::IGpioWrapper> gpio_wrapper,
                                         const uint8_t power_open_pin_id,
                                         const uint8_t power_close_pin_id)
      : _gpio_wrapper{gpio_wrapper}, _power_open_pin_id{power_open_pin_id}, _power_close_pin_id{power_close_pin_id}
  {
    initialize_lock();
  }

  void ElectricalTrayLock::initialize_lock()
  {
    // It is important to set this to low before configuring the pin mode, because the default output state is high
    set_lock_output_low();

    _gpio_wrapper->set_pin_mode(_power_open_pin_id, interfaces::gpio::IS_OUTPUT);
    _gpio_wrapper->set_pin_mode(_power_close_pin_id, interfaces::gpio::IS_OUTPUT);

    // on startup we assume the lock was open in the previous step to make sure the
    // lock will be closed when the set the lock to closed in the current step
    _lock_state_previous_step = lock::LockState::unlocked;
    set_expected_lock_state_current_step(lock::LockState::locked);
    set_is_tray_lock_opening_in_progress(false);
  }

  void ElectricalTrayLock::update_state()
  {
    // Mind that the state for open_lock_current_step_ is changed in the handle_lock_status function when a CAN msg is
    // received
    const bool change_lock_state = _expected_lock_state_current_step != _lock_state_previous_step;

    const unsigned long current_timestamp = millis();
    const unsigned long time_since_lock_state_was_changed = current_timestamp - _timestamp_last_lock_change;
    const bool has_lock_mechanism_time_passed =
      time_since_lock_state_was_changed >= _ELECTRICAL_LOCK_MECHANISM_TIME_IN_MS;

    if (change_lock_state && has_lock_mechanism_time_passed)
    {
      _lock_state_previous_step = _expected_lock_state_current_step;
      _timestamp_last_lock_change = current_timestamp;
      _expected_lock_state_current_step == lock::LockState::unlocked ? open_lock() : close_lock();
    }
    else if (!change_lock_state && has_lock_mechanism_time_passed)
    {
      // this makes sure, there is only a 5V pulse with the duration of ELECTRICAL_LOCK_MECHANISM_TIME_IN_MS on the
      // respective input of the lock
      set_lock_output_low();
    }
  }

  void ElectricalTrayLock::set_expected_lock_state_current_step(const lock::LockState expected_lock_state_current_step)
  {
    _expected_lock_state_current_step = expected_lock_state_current_step;
  }

  void ElectricalTrayLock::set_is_tray_lock_opening_in_progress(const bool is_tray_lock_opening_in_progress)
  {
    _is_tray_lock_opening_in_progress = is_tray_lock_opening_in_progress;
  }

  bool ElectricalTrayLock::is_tray_lock_opening_in_progress()
  {
    return _is_tray_lock_opening_in_progress;
  }

  void ElectricalTrayLock::set_is_drawer_opening_in_progress(const bool is_drawer_opening_in_progress)
  {
    _is_drawer_opening_in_progress = is_drawer_opening_in_progress;
  }

  bool ElectricalTrayLock::is_drawer_opening_in_progress()
  {
    return _is_drawer_opening_in_progress;
  }

  void ElectricalTrayLock::unlock()
  {
    if (is_tray_lock_opening_in_progress())
    {
      debug_printf("[ElectricalTrayLock]: Tray Lock opening is already in progress, so lock won't be opened again!\n");
    }
    else
    {
      set_expected_lock_state_current_step(lock::LockState::unlocked);
      set_is_tray_lock_opening_in_progress(true);
      set_is_drawer_opening_in_progress(true);
    }
  }

  void ElectricalTrayLock::open_lock()
  {
    _gpio_wrapper->digital_write(_power_close_pin_id, LOW);
    _gpio_wrapper->digital_write(_power_open_pin_id, HIGH);
  }

  void ElectricalTrayLock::close_lock()
  {
    _gpio_wrapper->digital_write(_power_open_pin_id, LOW);
    _gpio_wrapper->digital_write(_power_close_pin_id, HIGH);
  }

  void ElectricalTrayLock::set_lock_output_low()
  {
    _gpio_wrapper->digital_write(_power_open_pin_id, LOW);
    _gpio_wrapper->digital_write(_power_close_pin_id, LOW);
  }
}   // namespace tray