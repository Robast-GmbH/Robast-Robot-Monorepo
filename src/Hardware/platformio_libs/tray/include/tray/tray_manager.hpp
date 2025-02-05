#ifndef TRAY_TRAY_MANAGER_HPP
#define TRAY_TRAY_MANAGER_HPP

#include <memory>

#include "drawer/motion_controller.hpp"
#include "led/onboard_led_driver.hpp"
#include "motor/motor_monitor_config.hpp"
#include "switch/switch.hpp"
#include "tray/electrical_tray_lock.hpp"
#include "tray/tray_manager_config.hpp"
#include "tray/tray_pin_config.hpp"

namespace tray
{
  class TrayManager
  {
   public:
    TrayManager(const std::vector<TrayPinConfig>& tray_pin_configs,
                const std::shared_ptr<interfaces::IGpioWrapper> gpio_wrapper,
                const std::shared_ptr<TwoWire> wire,
                const std::shared_ptr<drawer::MotionController> motion_controller,
                const std::shared_ptr<motor::MotorMonitorConfig> motor_monitor_config,
                const std::shared_ptr<tray::TrayManagerConfig> tray_manager_config,
                const float switch_pressed_threshold,
                const float switch_new_reading_weight);

    void init(std::function<void()> set_enable_pin_high);

    void unlock_lock(uint8_t tray_id);

    void update_states();

    void set_tray_led_brightness(const uint8_t tray_id, const uint8_t led_row, const uint8_t brightness);

   private:
    const std::shared_ptr<drawer::MotionController> _motion_controller;
    const std::shared_ptr<motor::MotorMonitorConfig> _motor_monitor_config;
    const std::shared_ptr<tray::TrayManagerConfig> _tray_manager_config;

    std::vector<std::unique_ptr<ElectricalTrayLock>> _electrical_tray_locks;

    std::vector<std::unique_ptr<switch_lib::Switch>> _tray_switches;

    std::vector<uint32_t> _timestamp_last_tray_lock_opening;

    std::vector<bool> _reduced_speed_for_tray_lid;

    std::vector<uint8_t> _tray_lid_positions;

    uint8_t _target_speed_before_reduced_speed = 0;

    float _speed_deviation_in_percentage_for_stall_before_reduced_speed = 0.0f;

    const std::unique_ptr<led::OnboardLedDriver> _onboard_led_driver;

    bool _triggered_closing_lock_after_opening = false;

    static constexpr uint16_t _ELECTRICAL_TRAY_LOCK_MECHANISM_TIME_IN_MS = 700;

    uint8_t get_tray_lid_position(uint8_t tray_id);

    void update_tray_states();

    void handle_tray_just_opened(uint8_t vector_id);

    void handle_e_drawer_movement_to_close_lid(uint8_t tray_id);
  };

}   // namespace tray

#endif   // TRAY_TRAY_MANAGER_HPP