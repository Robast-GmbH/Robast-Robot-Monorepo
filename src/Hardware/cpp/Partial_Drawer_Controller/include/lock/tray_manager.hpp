#ifndef PARTIAL_DRAWER_CONTROLLER_TRAY_MANAGER_HPP
#define PARTIAL_DRAWER_CONTROLLER_TRAY_MANAGER_HPP

#include <memory>

#include "lock/electrical_tray_lock.hpp"
#include "lock/tray_pin_config.hpp"

namespace partial_drawer_controller
{

  class TrayManager
  {
   public:
    TrayManager(const std::vector<TrayPinConfig>& tray_pin_configs,
                std::shared_ptr<drawer_controller::IGpioWrapper> gpio_wrapper);

    void init_electrical_locks();

    void handle_electrical_lock_control();

   private:
    std::vector<std::unique_ptr<ElectricalTrayLock>> _electrical_tray_locks;
  };

}   // namespace partial_drawer_controller

#endif   // PARTIAL_DRAWER_CONTROLLER_TRAY_MANAGER_HPP