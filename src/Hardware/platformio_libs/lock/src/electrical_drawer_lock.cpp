#include "lock/electrical_drawer_lock.hpp"

namespace lock
{
  ElectricalDrawerLock::ElectricalDrawerLock(const std::shared_ptr<interfaces::IGpioWrapper> gpio_wrapper,
                                             const uint8_t power_open_pin_id,
                                             const uint8_t power_close_pin_id,
                                             const uint8_t sensor_lock_pin_id,
                                             const float switch_pressed_threshold,
                                             const float switch_new_reading_weight)
      : _gpio_wrapper{gpio_wrapper},
        _power_open_pin_id{power_open_pin_id},
        _power_close_pin_id{power_close_pin_id},
        _sensor_lock_pin_id{sensor_lock_pin_id},
        _switch_pressed_threshold{switch_pressed_threshold},
        _switch_new_reading_weight{switch_new_reading_weight}
  {
    initialize_lock();
  }

  void ElectricalDrawerLock::initialize_lock()
  {
    // It is important to set this to low before configuring the pin mode, because the default output state is high
    set_lock_output_low();

    _gpio_wrapper->set_pin_mode(_power_open_pin_id, interfaces::gpio::IS_OUTPUT);
    _gpio_wrapper->set_pin_mode(_power_close_pin_id, interfaces::gpio::IS_OUTPUT);
    _gpio_wrapper->set_pin_mode(_sensor_lock_pin_id, interfaces::gpio::IS_INPUT);

    _lock_state_previous_step = LockState::locked;
    set_expected_lock_state_current_step(LockState::locked);
    set_is_drawer_opening_in_progress(false);
    set_drawer_auto_close_timeout_triggered(false);
  }

  void ElectricalDrawerLock::handle_lock_control()
  {
    // Mind that the expected lock state is changed in the handle_lock_status function when a CAN msg is received
    const bool change_lock_state = _expected_lock_state_current_step != _lock_state_previous_step;

    const unsigned long current_timestamp = millis();
    const unsigned long time_since_lock_state_was_changed = current_timestamp - _timestamp_last_lock_change;
    const unsigned long time_since_lock_was_opened = current_timestamp - _timestamp_last_lock_opening;
    const bool has_lock_mechanism_time_passed =
      time_since_lock_state_was_changed >= _ELECTRICAL_LOCK_MECHANISM_TIME_IN_MS;

    if (change_lock_state && has_lock_mechanism_time_passed)
    {
      _lock_state_previous_step = _expected_lock_state_current_step;
      _timestamp_last_lock_change = current_timestamp;
      _expected_lock_state_current_step == LockState::unlocked ? open_lock() : close_lock();
    }
    else if (!change_lock_state && has_lock_mechanism_time_passed)
    {
      // this makes sure, there is only a 5V pulse with the duration of ELECTRICAL_LOCK_MECHANISM_TIME_IN_MS on the
      // respective input of the lock
      set_lock_output_low();
    }

    if (_expected_lock_state_current_step == LockState::unlocked &&
        time_since_lock_was_opened > _ELECTRICAL_LOCK_AUTO_CLOSE_TIME_WHEN_DRAWER_NOT_OPENED_IN_MS) 
    {
      // Close the lock automatically after some seconds when drawer wasn't opened for safety reasons
      set_expected_lock_state_current_step(LockState::locked);
      set_is_drawer_opening_in_progress(false);
      set_drawer_auto_close_timeout_triggered(true);
      debug_println(
        "[ElectricalDrawerLock]: Lock was automatically closed due to timeout! time_since_lock_was_opened: ");
      debug_println_with_base(time_since_lock_was_opened, DEC);
    }
  }

  void ElectricalDrawerLock::set_drawer_auto_close_timeout_triggered(const bool state)
  {
    _is_drawer_auto_close_timeout_triggered = state;
  }

  bool ElectricalDrawerLock::is_drawer_auto_close_timeout_triggered()
  {
    return _is_drawer_auto_close_timeout_triggered;
  }

  void ElectricalDrawerLock::set_expected_lock_state_current_step(const LockState expected_lock_state_current_step)
  {
    _expected_lock_state_current_step = expected_lock_state_current_step;
  }

  void ElectricalDrawerLock::set_timestamp_last_lock_change()
  {
    _timestamp_last_lock_opening = millis();
  }

  void ElectricalDrawerLock::set_is_drawer_opening_in_progress(const bool drawer_opening_is_in_progress)
  {
    _is_drawer_opening_in_progress = drawer_opening_is_in_progress;
  }

  bool ElectricalDrawerLock::is_drawer_opening_in_progress() const
  {
    return _is_drawer_opening_in_progress;
  }

  bool ElectricalDrawerLock::is_lock_switch_pushed() const
  {
    return get_moving_average_sensor_lock_pin() > _switch_pressed_threshold;
  }

  void ElectricalDrawerLock::update_sensor_values()
  {
    // Tracking the moving average for the sensor pins helps to debounce them a little bit
    byte digital_read_sensor_lock_pin;
    _gpio_wrapper->digital_read(_sensor_lock_pin_id, digital_read_sensor_lock_pin);
    _moving_average_sensor_lock_pin = _switch_new_reading_weight * digital_read_sensor_lock_pin +
                                      (1 - _switch_new_reading_weight) * _moving_average_sensor_lock_pin;
  }

  float ElectricalDrawerLock::get_moving_average_sensor_lock_pin() const
  {
    return _moving_average_sensor_lock_pin;
  }

  void ElectricalDrawerLock::unlock()
  {
    if (is_drawer_opening_in_progress())
    {
      debug_printf("[ElectricalDrawerLock]: Drawer opening is already in progress, so lock won't be opened again!\n");
    }
    else
    {
      set_expected_lock_state_current_step(LockState::unlocked);
      set_timestamp_last_lock_change();
      set_is_drawer_opening_in_progress(true);
    }
  }

  void ElectricalDrawerLock::open_lock() const
  {
    _gpio_wrapper->digital_write(_power_close_pin_id, LOW);
    _gpio_wrapper->digital_write(_power_open_pin_id, HIGH);
  }

  void ElectricalDrawerLock::close_lock() const
  {
    _gpio_wrapper->digital_write(_power_open_pin_id, LOW);
    _gpio_wrapper->digital_write(_power_close_pin_id, HIGH);
  }

  void ElectricalDrawerLock::set_lock_output_low() const
  {
    _gpio_wrapper->digital_write(_power_open_pin_id, LOW);
    _gpio_wrapper->digital_write(_power_close_pin_id, LOW);
  }
}   // namespace lock