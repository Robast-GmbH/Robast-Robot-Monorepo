#if !defined(DRAWER_CONTROLLER_LOCK_HPP)
#define DRAWER_CONTROLLER_LOCK_HPP

#include <Arduino.h>

#include "i_gpio_wrapper.hpp"

// the time in ms the lock mechanism needs to open resp. close the lock
#define LOCK_MECHANISM_TIME                         700     // according to the datasheet a minimum of 600ms is required
#define LOCK_AUTO_CLOSE_TIME_WHEN_DRAWER_NOT_OPENED 10000   // milliseconds

namespace drawer_controller
{
  class Lock
  {
   public:
    Lock(std::shared_ptr<IGpioWrapper> gpio_wrapper) : _gpio_wrapper{gpio_wrapper}
    {
    }

    void initialize_lock(uint8_t power_open_pin_id,
                         uint8_t power_close_pin_id,
                         uint8_t sensor_lock_pin_id,
                         uint8_t sensor_drawer_closed_pin_id,
                         bool gpio_input_state,
                         bool gpio_output_state)
    {
      _power_open_pin_id = power_open_pin_id;
      _power_close_pin_id = power_close_pin_id;
      _sensor_lock_pin_id = sensor_lock_pin_id;
      _sensor_drawer_closed_pin = sensor_drawer_closed_pin_id;

      _gpio_wrapper->set_pin_mode(_power_open_pin_id, gpio_output_state);
      _gpio_wrapper->set_pin_mode(_power_close_pin_id, gpio_output_state);
      _gpio_wrapper->set_pin_mode(_sensor_lock_pin_id, gpio_input_state);
      _gpio_wrapper->set_pin_mode(_sensor_drawer_closed_pin, gpio_input_state);

      _gpio_wrapper->digital_write(_power_open_pin_id, LOW);
      _gpio_wrapper->digital_write(_power_close_pin_id, LOW);
    }

    void handle_lock_control()
    {
      // Mind that the state for open_lock_current_step_ is changed in the handle_lock_status function when a CAN msg is
      // received
      bool change_lock_state = _open_lock_current_step == _open_lock_previous_step ? false : true;

      unsigned long current_timestamp = millis();
      unsigned long time_since_lock_state_was_changed = current_timestamp - _timestamp_last_lock_change;
      unsigned long time_since_lock_was_opened = current_timestamp - _timestamp_last_lock_opening;

      if (change_lock_state && (time_since_lock_state_was_changed >= LOCK_MECHANISM_TIME))
      {
        _open_lock_previous_step = _open_lock_current_step;
        _timestamp_last_lock_change = current_timestamp;
        _open_lock_current_step ? open_lock() : close_lock();
      }
      else if (!change_lock_state && (time_since_lock_state_was_changed >= LOCK_MECHANISM_TIME))
      {
        // this makes sure, there is only a 5V pulse with the duration of LOCK_MECHANISM_TIME on the respective input of
        // the lock
        set_lock_output_low();
      }

      if (_open_lock_current_step && (time_since_lock_was_opened > LOCK_AUTO_CLOSE_TIME_WHEN_DRAWER_NOT_OPENED))
      {
        // Close the lock automatically after some seconds when drawer wasn't opened for safety reasons
        set_open_lock_current_step(false);
        set_drawer_opening_is_in_progress(false);
        Serial.print(" time_since_lock_was_opened: ");
        Serial.println(time_since_lock_was_opened, DEC);
      }
    }

    void set_open_lock_current_step(bool open_lock_current_step)
    {
      _open_lock_current_step = open_lock_current_step;
    }

    void set_timestamp_last_lock_change()
    {
      _timestamp_last_lock_opening = millis();
    }

    void set_drawer_opening_is_in_progress(bool drawer_opening_is_in_progress)
    {
      _drawer_opening_is_in_progress = drawer_opening_is_in_progress;
    }

    bool is_drawer_opening_in_progress()
    {
      return _drawer_opening_is_in_progress;
    }

    bool is_lock_switch_pushed()
    {
      return get_moving_average_sensor_lock_pin() > 0.9;
    }

    bool is_endstop_switch_pushed()
    {
      return get_moving_average_drawer_closed_pin() > 0.9;
    }

    void handle_reading_sensors()
    {
      // Tracking the moving average for the sensor pins helps to debounce them a little bit
      byte digital_read_sensor_lock_pin;
      _gpio_wrapper->digital_read(_sensor_lock_pin_id, digital_read_sensor_lock_pin);
      _moving_average_sensor_lock_pin = 0.2 * digital_read_sensor_lock_pin + 0.8 * _moving_average_sensor_lock_pin;

      byte digital_drawer_closed_pin;
      _gpio_wrapper->digital_read(_sensor_drawer_closed_pin, digital_drawer_closed_pin);
      _moving_average_drawer_closed_pin = 0.2 * digital_drawer_closed_pin + 0.8 * _moving_average_drawer_closed_pin;
    }

    float get_moving_average_sensor_lock_pin()
    {
      return _moving_average_sensor_lock_pin;
    }

    float get_moving_average_drawer_closed_pin()
    {
      return _moving_average_drawer_closed_pin;
    }

   private:
    uint8_t _power_open_pin_id;
    uint8_t _power_close_pin_id;
    uint8_t _sensor_lock_pin_id;
    uint8_t _sensor_drawer_closed_pin;

    std::shared_ptr<IGpioWrapper> _gpio_wrapper;

    bool _open_lock_current_step = false;    // flag to store which state the locks should have
    bool _open_lock_previous_step = false;   // flag to store state of the lock of the previous step

    bool _drawer_opening_is_in_progress = false;

    unsigned long _timestamp_last_lock_change = 0;
    unsigned long _timestamp_last_lock_opening = 0;

    float _moving_average_sensor_lock_pin = 0;
    float _moving_average_drawer_closed_pin = 0;

    void open_lock()
    {
      _gpio_wrapper->digital_write(_power_close_pin_id, LOW);
      _gpio_wrapper->digital_write(_power_open_pin_id, HIGH);
    }

    void close_lock()
    {
      _gpio_wrapper->digital_write(_power_open_pin_id, LOW);
      _gpio_wrapper->digital_write(_power_close_pin_id, HIGH);
    }

    void set_lock_output_low()
    {
      _gpio_wrapper->digital_write(_power_open_pin_id, LOW);
      _gpio_wrapper->digital_write(_power_close_pin_id, LOW);
    }
  };
}   // namespace drawer_controller
#endif   // DRAWER_CONTROLLER_LOCK_HPP