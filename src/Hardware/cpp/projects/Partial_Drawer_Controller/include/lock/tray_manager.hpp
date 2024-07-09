#ifndef PARTIAL_DRAWER_CONTROLLER_TRAY_MANAGER_HPP
#define PARTIAL_DRAWER_CONTROLLER_TRAY_MANAGER_HPP

#include <memory>

#include "led/onboard_led_driver.hpp"
#include "lock/electrical_tray_lock.hpp"
#include "lock/tray_pin_config.hpp"
#include "switch/switch.hpp"

namespace partial_drawer_controller
{

  class TrayManager
  {
   public:
    TrayManager(const std::vector<TrayPinConfig>& tray_pin_configs,
                const std::shared_ptr<drawer_controller::IGpioWrapper> gpio_wrapper,
                const std::shared_ptr<TwoWire> wire,
                const float switch_pressed_threshold);

    void init(std::function<void()> set_enable_pin_high);

    void unlock_lock(uint8_t tray_id);

    void update_states();

    void set_tray_led_brightness(const uint8_t tray_id, const uint8_t led_row, const uint8_t brightness);

   private:
    std::vector<std::unique_ptr<ElectricalTrayLock>> _electrical_tray_locks;

    std::vector<std::unique_ptr<drawer_controller::Switch>> _tray_switches;

    const std::unique_ptr<OnboardLedDriver> _onboard_led_driver;

    bool _triggered_closing_lock_after_opening = false;

    void handle_tray_just_opened(uint8_t vector_id);
  };

}   // namespace partial_drawer_controller

#endif   // PARTIAL_DRAWER_CONTROLLER_TRAY_MANAGER_HPP