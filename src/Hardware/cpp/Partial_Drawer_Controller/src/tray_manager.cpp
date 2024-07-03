#include "lock/tray_manager.hpp"

namespace partial_drawer_controller
{
  TrayManager::TrayManager(const std::vector<TrayPinConfig>& tray_pin_configs,
                           std::shared_ptr<drawer_controller::IGpioWrapper> gpio_wrapper,
                           std::shared_ptr<TwoWire> wire)
      : _onboard_led_driver{std::make_unique<OnboardLedDriver>(wire)}
  {
    _electrical_tray_locks.reserve(tray_pin_configs.size());
    for (const auto& tray_pin_config : tray_pin_configs)
    {
      _electrical_tray_locks.push_back(std::make_unique<ElectricalTrayLock>(
        gpio_wrapper, tray_pin_config.power_open_pin_id, tray_pin_config.power_close_pin_id));
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

  void TrayManager::set_tray_led_brightness(uint8_t tray_id, uint8_t brightness)
  {
    _onboard_led_driver->set_tray_led_brightness(tray_id, brightness);
  }

  void TrayManager::update_states()
  {
    for (const auto& electrical_tray_lock : _electrical_tray_locks)
    {
      electrical_tray_lock->update_state();
    }
  }
}   // namespace partial_drawer_controller