#ifndef PARTIAL_DRAWER_CONTROLLER_ELECTRICAL_TRAY_LOCK_HPP
#define PARTIAL_DRAWER_CONTROLLER_ELECTRICAL_TRAY_LOCK_HPP

#include <Arduino.h>

#include <memory>

#include "debug/debug.hpp"
#include "interfaces/i_gpio_wrapper.hpp"

// the time in ms the lock mechanism needs to open resp. close the lock
#define ELECTRICAL_LOCK_MECHANISM_TIME_IN_MS 700   // according to the datasheet a minimum of 600ms is required

namespace partial_drawer_controller
{
  class ElectricalTrayLock
  {
   public:
    ElectricalTrayLock(const std::shared_ptr<interfaces::IGpioWrapper> gpio_wrapper,
                       const uint8_t power_open_pin_id,
                       const uint8_t power_close_pin_id);

    void initialize_lock();

    void update_state();

    void set_open_lock_current_step(const bool open_lock_current_step);

    void set_drawer_opening_is_in_progress(const bool drawer_opening_is_in_progress);

    bool is_drawer_opening_in_progress();

    void unlock();

   private:
    const uint8_t _power_open_pin_id;
    const uint8_t _power_close_pin_id;

    const std::shared_ptr<interfaces::IGpioWrapper> _gpio_wrapper;

    bool _open_lock_current_step;    // flag to store which state the locks should have
    bool _open_lock_previous_step;   // flag to store state of the lock of the previous step

    bool _drawer_opening_is_in_progress = false;

    unsigned long _timestamp_last_lock_change = 0;

    void open_lock();

    void close_lock();

    void set_lock_output_low();
  };
}   // namespace partial_drawer_controller
#endif   // PARTIAL_DRAWER_CONTROLLER_ELECTRICAL_TRAY_LOCK_HPP