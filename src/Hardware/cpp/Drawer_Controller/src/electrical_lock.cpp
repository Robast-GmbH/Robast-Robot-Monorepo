#include "electrical_lock.hpp"

namespace drawer_controller
{
  ElectricalLock::ElectricalLock(std::shared_ptr<IGpioWrapper> gpio_wrapper) : _gpio_wrapper{gpio_wrapper}
  {
  }

  void ElectricalLock::initialize_lock(uint8_t power_open_pin_id,
                                       uint8_t power_close_pin_id,
                                       uint8_t sensor_lock_pin_id,
                                       uint8_t sensor_drawer_closed_pin_id)
  {
    _power_open_pin_id = power_open_pin_id;
    _power_close_pin_id = power_close_pin_id;
    _sensor_lock_pin_id = sensor_lock_pin_id;
    _sensor_drawer_closed_pin = sensor_drawer_closed_pin_id;

    _gpio_wrapper->set_pin_mode(_power_open_pin_id, _gpio_wrapper->get_gpio_output_pin_mode());
    _gpio_wrapper->set_pin_mode(_power_close_pin_id, _gpio_wrapper->get_gpio_output_pin_mode());
    _gpio_wrapper->set_pin_mode(_sensor_lock_pin_id, _gpio_wrapper->get_gpio_input_pin_mode());
    _gpio_wrapper->set_pin_mode(_sensor_drawer_closed_pin, _gpio_wrapper->get_gpio_input_pin_mode());

    _gpio_wrapper->digital_write(_power_open_pin_id, LOW);
    _gpio_wrapper->digital_write(_power_close_pin_id, LOW);

    _open_lock_previous_step = false;
    set_open_lock_current_step(false);
    set_drawer_opening_is_in_progress(false);
    set_drawer_auto_close_timeout_triggered(false);
  }

  void ElectricalLock::handle_lock_control()
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
      debug_println("Lock was automatically closed due to timeout! time_since_lock_was_opened: ");
      debug_println_with_base(time_since_lock_was_opened, DEC);
    }
  }

  void ElectricalLock::set_drawer_auto_close_timeout_triggered(bool state)
  {
    _is_drawer_auto_close_timeout_triggered = state;
  }

  bool ElectricalLock::is_drawer_auto_close_timeout_triggered()
  {
    return _is_drawer_auto_close_timeout_triggered;
  }

  void ElectricalLock::set_open_lock_current_step(bool open_lock_current_step)
  {
    _open_lock_current_step = open_lock_current_step;
  }

  void ElectricalLock::set_timestamp_last_lock_change()
  {
    _timestamp_last_lock_opening = millis();
  }

  void ElectricalLock::set_drawer_opening_is_in_progress(bool drawer_opening_is_in_progress)
  {
    _drawer_opening_is_in_progress = drawer_opening_is_in_progress;
  }

  bool ElectricalLock::is_drawer_opening_in_progress()
  {
    return _drawer_opening_is_in_progress;
  }

  bool ElectricalLock::is_lock_switch_pushed()
  {
    return get_moving_average_sensor_lock_pin() > 0.9;
  }

  bool ElectricalLock::is_endstop_switch_pushed()
  {
    return get_moving_average_drawer_closed_pin() > 0.9;
  }

  void ElectricalLock::handle_reading_sensors()
  {
    // Tracking the moving average for the sensor pins helps to debounce them a little bit
    byte digital_read_sensor_lock_pin;
    _gpio_wrapper->digital_read(_sensor_lock_pin_id, digital_read_sensor_lock_pin);
    _moving_average_sensor_lock_pin = 0.2 * digital_read_sensor_lock_pin + 0.8 * _moving_average_sensor_lock_pin;

    byte digital_drawer_closed_pin;
    _gpio_wrapper->digital_read(_sensor_drawer_closed_pin, digital_drawer_closed_pin);
    _moving_average_drawer_closed_pin = 0.2 * digital_drawer_closed_pin + 0.8 * _moving_average_drawer_closed_pin;
  }

  float ElectricalLock::get_moving_average_sensor_lock_pin()
  {
    return _moving_average_sensor_lock_pin;
  }

  float ElectricalLock::get_moving_average_drawer_closed_pin()
  {
    return _moving_average_drawer_closed_pin;
  }

  void ElectricalLock::unlock(uint8_t id)
  {
    if (is_drawer_opening_in_progress())
    {
      debug_printf("Drawer%d opening is already in progress, so lock won't be opened again!\n", id);
    }
    else
    {
      set_open_lock_current_step(true);
      set_timestamp_last_lock_change();
      set_drawer_opening_is_in_progress(true);
    }
  }

  void ElectricalLock::open_lock()
  {
    _gpio_wrapper->digital_write(_power_close_pin_id, LOW);
    _gpio_wrapper->digital_write(_power_open_pin_id, HIGH);
  }

  void ElectricalLock::close_lock()
  {
    _gpio_wrapper->digital_write(_power_open_pin_id, LOW);
    _gpio_wrapper->digital_write(_power_close_pin_id, HIGH);
  }

  void ElectricalLock::set_lock_output_low()
  {
    _gpio_wrapper->digital_write(_power_open_pin_id, LOW);
    _gpio_wrapper->digital_write(_power_close_pin_id, LOW);
  }
}   // namespace drawer_controller