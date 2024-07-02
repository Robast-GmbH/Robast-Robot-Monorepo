#include "lock/tray_manager.hpp"

namespace partial_drawer_controller
{
  TrayManager::TrayManager(const std::vector<TrayPinConfig>& tray_pin_configs,
                           std::shared_ptr<drawer_controller::IGpioWrapper> gpio_wrapper)
  {
    _electrical_tray_locks.reserve(tray_pin_configs.size());
    for (const auto& tray_pin_config : tray_pin_configs)
    {
      _electrical_tray_locks.push_back(std::make_unique<ElectricalTrayLock>(
        gpio_wrapper, tray_pin_config.power_open_pin_id, tray_pin_config.power_close_pin_id));
    }
  }

  void TrayManager::init_electrical_locks()
  {
    for (const auto& electrical_tray_lock : _electrical_tray_locks)
    {
      electrical_tray_lock->initialize_lock();
    }
  }

  void TrayManager::handle_electrical_lock_control()
  {
    // TODO: Implement this function
  }
}   // namespace partial_drawer_controller