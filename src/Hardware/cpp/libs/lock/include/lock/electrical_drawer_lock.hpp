#ifndef LOCK_ELECTRICAL_DRAWER_LOCK_HPP
#define LOCK_ELECTRICAL_DRAWER_LOCK_HPP

#include <Arduino.h>

#include <memory>

#include "debug/debug.hpp"
#include "interfaces/i_gpio_wrapper.hpp"

// the time in ms the lock mechanism needs to open resp. close the lock
#define ELECTRICAL_LOCK_MECHANISM_TIME                         700   // according to the datasheet a minimum of 600ms is required
#define ELECTRICAL_LOCK_AUTO_CLOSE_TIME_WHEN_DRAWER_NOT_OPENED 10000   // milliseconds

namespace lock
{
  class ElectricalDrawerLock
  {
   public:
    ElectricalDrawerLock(const std::shared_ptr<interfaces::IGpioWrapper> gpio_wrapper,
                         const uint8_t power_open_pin_id,
                         const uint8_t power_close_pin_id,
                         const uint8_t sensor_lock_pin_id,
                         const float switch_pressed_threshold,
                         const float switch_new_reading_weight);

    void initialize_lock();

    void handle_lock_control();

    void set_open_lock_current_step(const bool open_lock_current_step);

    void set_timestamp_last_lock_change();

    void set_drawer_opening_is_in_progress(const bool drawer_opening_is_in_progress);

    bool is_drawer_opening_in_progress() const;

    bool is_lock_switch_pushed() const;

    void update_sensor_values();

    float get_moving_average_sensor_lock_pin() const;

    void unlock();

    void set_drawer_auto_close_timeout_triggered(const bool state);

    bool is_drawer_auto_close_timeout_triggered();

   private:
    const uint8_t _power_open_pin_id;
    const uint8_t _power_close_pin_id;
    const uint8_t _sensor_lock_pin_id;
    const float _switch_pressed_threshold;
    const float _switch_new_reading_weight;

    const std::shared_ptr<interfaces::IGpioWrapper> _gpio_wrapper;

    bool _open_lock_current_step;    // flag to store which state the locks should have
    bool _open_lock_previous_step;   // flag to store state of the lock of the previous step

    bool _drawer_opening_is_in_progress = false;

    bool _is_drawer_auto_close_timeout_triggered = false;

    unsigned long _timestamp_last_lock_change = 0;
    unsigned long _timestamp_last_lock_opening = 0;

    float _moving_average_sensor_lock_pin = 0;

    void open_lock() const;

    void close_lock() const;

    void set_lock_output_low() const;
  };
}   // namespace lock
#endif   // LOCK_ELECTRICAL_DRAWER_LOCK_HPP