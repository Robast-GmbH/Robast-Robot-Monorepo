#include "lock/electrical_tray_lock.hpp"

namespace partial_drawer_controller
{
  ElectricalTrayLock::ElectricalTrayLock(const std::shared_ptr<drawer_controller::IGpioWrapper> gpio_wrapper,
                                         const uint8_t power_open_pin_id,
                                         const uint8_t power_close_pin_id)
      : _gpio_wrapper{gpio_wrapper}, _power_open_pin_id{power_open_pin_id}, _power_close_pin_id{power_close_pin_id}
  {
  }

  void ElectricalTrayLock::initialize_lock()
  {
    // It is important to set this to low before configuring the pin mode, because the default output state is high
    set_lock_output_low();

    _gpio_wrapper->set_pin_mode(_power_open_pin_id, _gpio_wrapper->get_gpio_output_pin_mode());
    _gpio_wrapper->set_pin_mode(_power_close_pin_id, _gpio_wrapper->get_gpio_output_pin_mode());

    // on startup we assume the lock was open in the previous step to make sure the
    // lock will be closed when the set the lock to closed in the current step
    _open_lock_previous_step = true;
    set_open_lock_current_step(false);
    set_drawer_opening_is_in_progress(false);
  }

  void ElectricalTrayLock::update_state()
  {
    // Mind that the state for open_lock_current_step_ is changed in the handle_lock_status function when a CAN msg is
    // received
    const bool change_lock_state = !(_open_lock_current_step == _open_lock_previous_step);

    const unsigned long current_timestamp = millis();
    const unsigned long time_since_lock_state_was_changed = current_timestamp - _timestamp_last_lock_change;

    if (change_lock_state && (time_since_lock_state_was_changed >= ELECTRICAL_LOCK_MECHANISM_TIME))
    {
      _open_lock_previous_step = _open_lock_current_step;
      _timestamp_last_lock_change = current_timestamp;
      _open_lock_current_step ? open_lock() : close_lock();
    }
    else if (!change_lock_state && (time_since_lock_state_was_changed >= ELECTRICAL_LOCK_MECHANISM_TIME))
    {
      // this makes sure, there is only a 5V pulse with the duration of ELECTRICAL_LOCK_MECHANISM_TIME on the respective
      // input of the lock
      set_lock_output_low();
    }
  }

  void ElectricalTrayLock::set_open_lock_current_step(const bool open_lock_current_step)
  {
    _open_lock_current_step = open_lock_current_step;
  }

  void ElectricalTrayLock::set_drawer_opening_is_in_progress(const bool drawer_opening_is_in_progress)
  {
    _drawer_opening_is_in_progress = drawer_opening_is_in_progress;
  }

  bool ElectricalTrayLock::is_drawer_opening_in_progress()
  {
    return _drawer_opening_is_in_progress;
  }

  void ElectricalTrayLock::unlock()
  {
    if (is_drawer_opening_in_progress())
    {
      debug_printf("[ElectricalTrayLock]: Drawer opening is already in progress, so lock won't be opened again!\n");
    }
    else
    {
      set_open_lock_current_step(true);
      set_drawer_opening_is_in_progress(true);
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
}   // namespace partial_drawer_controller