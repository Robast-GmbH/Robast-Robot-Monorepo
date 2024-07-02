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

  void TrayManager::handle_electrical_lock_control()
  {
    // TODO: Implement this function
  }
}   // namespace partial_drawer_controller