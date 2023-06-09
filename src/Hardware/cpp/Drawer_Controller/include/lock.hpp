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
    Lock()
    {
    }

    void initialize_lock(uint8_t power_open_pin_id,
                         uint8_t power_close_pin_id,
                         uint8_t sensor_lock_pin_id,
                         uint8_t sensor_drawer_closed_pin_id,
                         std::unique_ptr<IGpioWrapper> gpio_wrapper)
    {
      power_open_pin_id_ = power_open_pin_id;
      power_close_pin_id_ = power_close_pin_id;
      sensor_lock_pin_id_ = sensor_lock_pin_id;
      sensor_drawer_closed_pin_ = sensor_drawer_closed_pin_id;

      gpio_wrapper_ = std::move(gpio_wrapper);

      gpio_wrapper_->set_pin_mode(power_open_pin_id_, OUTPUT);
      gpio_wrapper_->set_pin_mode(power_close_pin_id_, OUTPUT);
      gpio_wrapper_->set_pin_mode(sensor_lock_pin_id_, INPUT);
      gpio_wrapper_->set_pin_mode(sensor_drawer_closed_pin_, INPUT);

      gpio_wrapper_->digital_write(power_open_pin_id_, LOW);
      gpio_wrapper_->digital_write(power_close_pin_id_, LOW);
    }

    void handle_lock_control()
    {
      // Mind that the state for open_lock_current_step_ is changed in the handle_lock_status function when a CAN msg is
      // received
      bool change_lock_state = open_lock_current_step_ == open_lock_previous_step_ ? false : true;

      unsigned long current_timestamp = millis();
      unsigned long time_since_lock_state_was_changed = current_timestamp - timestamp_last_lock_change_;
      unsigned long time_since_lock_was_opened = current_timestamp - timestamp_last_lock_opening_;

      if (change_lock_state && (time_since_lock_state_was_changed >= LOCK_MECHANISM_TIME))
      {
        open_lock_previous_step_ = open_lock_current_step_;
        timestamp_last_lock_change_ = current_timestamp;
        open_lock_current_step_ ? open_lock() : close_lock();
      }
      else if (!change_lock_state && (time_since_lock_state_was_changed >= LOCK_MECHANISM_TIME))
      {
        // this makes sure, there is only a 5V pulse with the duration of LOCK_MECHANISM_TIME on the respective input of
        // the lock
        set_lock_output_low();
      }

      if (open_lock_current_step_ && (time_since_lock_was_opened > LOCK_AUTO_CLOSE_TIME_WHEN_DRAWER_NOT_OPENED))
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
      open_lock_current_step_ = open_lock_current_step;
    }

    void set_timestamp_last_lock_change()
    {
      timestamp_last_lock_opening_ = millis();
    }

    void set_drawer_opening_is_in_progress(bool drawer_opening_is_in_progress)
    {
      drawer_opening_is_in_progress_ = drawer_opening_is_in_progress;
    }

    bool is_drawer_opening_in_progress()
    {
      return drawer_opening_is_in_progress_;
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
      gpio_wrapper_->digital_read(sensor_lock_pin_id_, digital_read_sensor_lock_pin);
      moving_average_sensor_lock_pin_ = 0.2 * digital_read_sensor_lock_pin + 0.8 * moving_average_sensor_lock_pin_;

      byte digital_drawer_closed_pin;
      gpio_wrapper_->digital_read(sensor_drawer_closed_pin_, digital_drawer_closed_pin);
      moving_average_drawer_closed_pin_ = 0.2 * digital_drawer_closed_pin + 0.8 * moving_average_drawer_closed_pin_;
    }

    float get_moving_average_sensor_lock_pin()
    {
      return moving_average_sensor_lock_pin_;
    }

    float get_moving_average_drawer_closed_pin()
    {
      return moving_average_drawer_closed_pin_;
    }

   private:
    uint8_t power_open_pin_id_;
    uint8_t power_close_pin_id_;
    uint8_t sensor_lock_pin_id_;
    uint8_t sensor_drawer_closed_pin_;

    std::unique_ptr<IGpioWrapper> gpio_wrapper_;

    bool open_lock_current_step_ = false;    // flag to store which state the locks should have
    bool open_lock_previous_step_ = false;   // flag to store state of the lock of the previous step

    bool drawer_opening_is_in_progress_ = false;

    unsigned long timestamp_last_lock_change_ = 0;
    unsigned long timestamp_last_lock_opening_ = 0;

    float moving_average_sensor_lock_pin_ = 0;
    float moving_average_drawer_closed_pin_ = 0;

    void open_lock()
    {
      gpio_wrapper_->digital_write(power_close_pin_id_, LOW);
      gpio_wrapper_->digital_write(power_open_pin_id_, HIGH);
    }

    void close_lock()
    {
      gpio_wrapper_->digital_write(power_open_pin_id_, LOW);
      gpio_wrapper_->digital_write(power_close_pin_id_, HIGH);
    }

    void set_lock_output_low()
    {
      gpio_wrapper_->digital_write(power_open_pin_id_, LOW);
      gpio_wrapper_->digital_write(power_close_pin_id_, LOW);
    }
  };
}   // namespace drawer_controller
#endif   // DRAWER_CONTROLLER_LOCK_HPP