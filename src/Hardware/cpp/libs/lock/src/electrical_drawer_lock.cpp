#include "lock/electrical_drawer_lock.hpp"

namespace drawer_controller
{
  ElectricalDrawerLock::ElectricalDrawerLock(const std::shared_ptr<IGpioWrapper> gpio_wrapper,
                                             const uint8_t power_open_pin_id,
                                             const uint8_t power_close_pin_id,
                                             const uint8_t sensor_lock_pin_id,
                                             const float switch_pressed_threshold,
                                             const float switch_weight_new_values)
      : _gpio_wrapper{gpio_wrapper},
        _power_open_pin_id{power_open_pin_id},
        _power_close_pin_id{power_close_pin_id},
        _sensor_lock_pin_id{sensor_lock_pin_id},
        _switch_pressed_threshold{switch_pressed_threshold},
        _switch_weight_new_values{switch_weight_new_values}
  {
  }

  void ElectricalDrawerLock::initialize_lock()
  {
    // It is important to set this to low before configuring the pin mode, because the default output state is high
    set_lock_output_low();

    _gpio_wrapper->set_pin_mode(_power_open_pin_id, _gpio_wrapper->get_gpio_output_pin_mode());
    _gpio_wrapper->set_pin_mode(_power_close_pin_id, _gpio_wrapper->get_gpio_output_pin_mode());
    _gpio_wrapper->set_pin_mode(_sensor_lock_pin_id, _gpio_wrapper->get_gpio_input_pin_mode());

    _open_lock_previous_step = false;
    set_open_lock_current_step(false);
    set_drawer_opening_is_in_progress(false);
    set_drawer_auto_close_timeout_triggered(false);
  }

  void ElectricalDrawerLock::handle_lock_control()
  {
    // Mind that the state for open_lock_current_step_ is changed in the handle_lock_status function when a CAN msg is
    // received
    bool change_lock_state = !(_open_lock_current_step == _open_lock_previous_step);

    unsigned long current_timestamp = millis();
    unsigned long time_since_lock_state_was_changed = current_timestamp - _timestamp_last_lock_change;
    unsigned long time_since_lock_was_opened = current_timestamp - _timestamp_last_lock_opening;

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

    if (_open_lock_current_step &&
        (time_since_lock_was_opened > ELECTRICAL_LOCK_AUTO_CLOSE_TIME_WHEN_DRAWER_NOT_OPENED))
    {
      // Close the lock automatically after some seconds when drawer wasn't opened for safety reasons
      set_open_lock_current_step(false);
      set_drawer_opening_is_in_progress(false);
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

  void ElectricalDrawerLock::set_open_lock_current_step(const bool open_lock_current_step)
  {
    _open_lock_current_step = open_lock_current_step;
  }

  void ElectricalDrawerLock::set_timestamp_last_lock_change()
  {
    _timestamp_last_lock_opening = millis();
  }

  void ElectricalDrawerLock::set_drawer_opening_is_in_progress(const bool drawer_opening_is_in_progress)
  {
    _drawer_opening_is_in_progress = drawer_opening_is_in_progress;
  }

  bool ElectricalDrawerLock::is_drawer_opening_in_progress() const
  {
    return _drawer_opening_is_in_progress;
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
    _moving_average_sensor_lock_pin = _switch_weight_new_values * digital_read_sensor_lock_pin +
                                      (1 - _switch_weight_new_values) * _moving_average_sensor_lock_pin;
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
      set_open_lock_current_step(true);
      set_timestamp_last_lock_change();
      set_drawer_opening_is_in_progress(true);
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
}   // namespace drawer_controller