#ifndef PARTIAL_DRAWER_CONTROLLER_TRAY_MANAGER_HPP
#define PARTIAL_DRAWER_CONTROLLER_TRAY_MANAGER_HPP

#include <memory>

#include "led/onboard_led_driver.hpp"
#include "lock/electrical_tray_lock.hpp"
#include "lock/tray_pin_config.hpp"

namespace partial_drawer_controller
{

  class TrayManager
  {
   public:
    TrayManager(const std::vector<TrayPinConfig>& tray_pin_configs,
                std::shared_ptr<drawer_controller::IGpioWrapper> gpio_wrapper,
                std::shared_ptr<TwoWire> wire);

    void init(std::function<void()> set_enable_pin_high);

    void handle_electrical_lock_control();

   private:
    std::vector<std::unique_ptr<ElectricalTrayLock>> _electrical_tray_locks;

    std::unique_ptr<OnboardLedDriver> _onboard_led_driver;
  };

}   // namespace partial_drawer_controller

#endif   // PARTIAL_DRAWER_CONTROLLER_TRAY_MANAGER_HPP