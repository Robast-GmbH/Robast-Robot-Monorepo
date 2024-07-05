#include "lock/tray_manager.hpp"

namespace partial_drawer_controller
{
  TrayManager::TrayManager(const std::vector<TrayPinConfig>& tray_pin_configs,
                           std::shared_ptr<drawer_controller::IGpioWrapper> gpio_wrapper,
                           std::shared_ptr<TwoWire> wire,
                           float switch_pressed_threshold)
      : _onboard_led_driver{std::make_unique<OnboardLedDriver>(wire)}
  {
    _electrical_tray_locks.reserve(tray_pin_configs.size());
    _tray_switches.reserve(tray_pin_configs.size());
    for (const auto& tray_pin_config : tray_pin_configs)
    {
      _electrical_tray_locks.push_back(std::make_unique<ElectricalTrayLock>(
        gpio_wrapper, tray_pin_config.power_open_pin_id, tray_pin_config.power_close_pin_id));
      _tray_switches.push_back(std::make_unique<drawer_controller::Switch>(gpio_wrapper,
                                                                           tray_pin_config.sensor_lock_pin_id,
                                                                           switch_pressed_threshold,
                                                                           drawer_controller::Switch::normally_open));
    }
  }

  void TrayManager::init(std::function<void()> set_enable_pin_high)
  {
    for (const auto& electrical_tray_lock : _electrical_tray_locks)
    {
      electrical_tray_lock->initialize_lock();
    }

    _onboard_led_driver->initialize(set_enable_pin_high);
  }

  void TrayManager::unlock_lock(uint8_t tray_id)
  {
    _electrical_tray_locks[tray_id - 1]->unlock();
  }

  void TrayManager::set_tray_led_brightness(uint8_t tray_id, uint8_t led_row, uint8_t brightness)
  {
    _onboard_led_driver->set_tray_led_brightness(tray_id, led_row, brightness);
  }

  void TrayManager::update_states()
  {
    for (uint8_t i = 0; i < _electrical_tray_locks.size(); ++i)
    {
      _electrical_tray_locks[i]->update_state();
      _tray_switches[i]->update_sensor_value();
      handle_tray_just_opened(i);
    }
  }

  void TrayManager::handle_tray_just_opened(uint8_t vector_id)
  {
    bool is_tray_open = !_tray_switches[vector_id]->is_switch_pressed();
    if (_electrical_tray_locks[vector_id]->is_drawer_opening_in_progress() && is_tray_open)
    {
      // this makes sure the lock automatically closes as soon as the drawer is opened
      _electrical_tray_locks[vector_id]->set_open_lock_current_step(false);

      debug_println("Triggered closing the lock because tray just openend!");
      _electrical_tray_locks[vector_id]->set_drawer_opening_is_in_progress(false);
    }
  }
}   // namespace partial_drawer_controller