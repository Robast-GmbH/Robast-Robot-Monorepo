#ifndef PARTIAL_DRAWER_CONTROLLER_TRAY_MANAGER_HPP
#define PARTIAL_DRAWER_CONTROLLER_TRAY_MANAGER_HPP

#include <memory>

#include "drawer/electrical_drawer.hpp"
#include "led/onboard_led_driver.hpp"
#include "lock/electrical_tray_lock.hpp"
#include "lock/tray_pin_config.hpp"
#include "motor/motor_monitor_config.hpp"
#include "switch/switch.hpp"

namespace partial_drawer_controller
{
  class TrayManager
  {
   public:
    TrayManager(const std::vector<TrayPinConfig>& tray_pin_configs,
                const std::shared_ptr<interfaces::IGpioWrapper> gpio_wrapper,
                const std::shared_ptr<TwoWire> wire,
                const std::shared_ptr<drawer::ElectricalDrawer> e_drawer,
                const std::shared_ptr<motor::MotorMonitorConfig> motor_monitor_config,
                const float switch_pressed_threshold,
                const float switch_new_reading_weight);

    void init(std::function<void()> set_enable_pin_high);

    void unlock_lock(uint8_t tray_id);

    void update_states();

    void set_tray_led_brightness(const uint8_t tray_id, const uint8_t led_row, const uint8_t brightness);

   private:
    const std::shared_ptr<drawer::ElectricalDrawer> _e_drawer;

    const std::shared_ptr<motor::MotorMonitorConfig> _motor_monitor_config;

    std::vector<std::unique_ptr<ElectricalTrayLock>> _electrical_tray_locks;

    std::vector<std::unique_ptr<switch_lib::Switch>> _tray_switches;

    std::vector<uint32_t> _timestamp_last_tray_lock_opening;

    std::vector<bool> _reduced_speed_for_tray_lid;

    uint8_t _target_speed_before_reduced_speed = 0;

    float _speed_deviation_in_percentage_for_stall_before_reduced_speed = 0.0;

    const std::unique_ptr<OnboardLedDriver> _onboard_led_driver;

    bool _triggered_closing_lock_after_opening = false;

    static constexpr uint16_t _ELECTRICAL_TRAY_LOCK_MECHANISM_TIME_IN_MS = 700;

    void update_tray_states();

    void handle_tray_just_opened(uint8_t vector_id);

    void handle_e_drawer_movement_to_close_lid(uint8_t tray_id);
  };

}   // namespace partial_drawer_controller

#endif   // PARTIAL_DRAWER_CONTROLLER_TRAY_MANAGER_HPP