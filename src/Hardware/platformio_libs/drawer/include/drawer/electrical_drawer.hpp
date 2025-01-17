#ifndef DRAWER_ELECTRICAL_DRAWER_HPP
#define DRAWER_ELECTRICAL_DRAWER_HPP

#include <Arduino.h>

#include <memory>
#include <optional>

#include "can_toolbox/can_utils.hpp"
#include "drawer/electrical_drawer_config.hpp"
#include "drawer/motion_controller.hpp"
#include "interfaces/i_drawer.hpp"
#include "interfaces/i_gpio_wrapper.hpp"
#include "lock/electrical_drawer_lock.hpp"
#include "motor/encoder.hpp"
#include "motor/encoder_monitor.hpp"
#include "motor/enconder_config.hpp"
#include "motor/motor.hpp"
#include "motor/motor_config.hpp"
#include "motor/motor_monitor.hpp"
#include "switch/switch.hpp"
#include "utils/e_drawer_task.hpp"

namespace drawer
{

  constexpr bool IS_HOMING = true;
  constexpr bool IS_NOT_HOMING = false;
  constexpr bool DO_NOT_USE_ACCELERATION_RAMP = false;
  constexpr bool PUSH_TO_CLOSE_TRIGGERED = true;
  constexpr bool MOTOR_IS_STALLED = true;
  constexpr bool ENDSTOP_SWITCH_IS_PUSHED = true;
  constexpr bool CONFIRM_MOTOR_CONTROL_CHANGE = true;

  class ElectricalDrawer : public interfaces::IDrawer
  {
  public:
    ElectricalDrawer(const uint32_t module_id,
                     const uint8_t id,
                     const std::shared_ptr<interfaces::IGpioWrapper> gpio_wrapper,
                     const std::shared_ptr<can_toolbox::CanUtils> can_utils,
                     const stepper_motor::StepperPinIdConfig &stepper_pin_id_config,
                     const bool use_encoder,
                     const uint8_t encoder_pin_a,
                     const uint8_t encoder_pin_b,
                     const uint8_t motor_driver_address,
                     const std::shared_ptr<motor::MotorConfig> motor_config,
                     const std::shared_ptr<switch_lib::Switch> endstop_switch,
                     const std::optional<std::shared_ptr<lock::ElectricalDrawerLock>> drawer_lock,
                     const std::shared_ptr<ElectricalDrawerConfig> e_drawer_config,
                     const std::shared_ptr<motor::EncoderConfig> encoder_config,
                     const std::shared_ptr<motor::MotorMonitorConfig> motor_monitor_config);

    void update_state() override;

    void unlock() override;

    void add_e_drawer_task_to_queue(const utils::EDrawerTask &e_drawer_task) override;

    void set_motor_driver_state(const bool enabled, const uint8_t motor_id) const override;

    uint8_t get_current_position() const;

    uint8_t get_target_speed() const;

    bool is_drawer_moving_in() const;

    void set_target_speed_and_direction(const uint8_t target_speed, const bool use_acceleration_ramp);

    void set_target_speed_with_decelerating_ramp(const uint8_t target_speed);

  private:
    const uint32_t _module_id;
    const uint8_t _id;

    const std::shared_ptr<interfaces::IGpioWrapper> _gpio_wrapper;

    const stepper_motor::StepperPinIdConfig _stepper_pin_id_config;

    const std::shared_ptr<motor::Encoder> _encoder;

    const std::shared_ptr<switch_lib::Switch> _endstop_switch;

    // optional because the lock is not always installed (e.g. in the partial drawer)
    const std::optional<std::shared_ptr<lock::ElectricalDrawerLock>> _drawer_lock;

    const std::shared_ptr<can_toolbox::CanUtils> _can_utils;

    const std::shared_ptr<stepper_motor::Motor> _motor;

    std::unique_ptr<utils::Queue<utils::EDrawerTask>> _e_drawer_task_queue =
        std::make_unique<utils::Queue<utils::EDrawerTask>>();

    const std::shared_ptr<ElectricalDrawerConfig> _config;

    const std::unique_ptr<motor::EncoderMonitor> _encoder_monitor;

    const std::unique_ptr<MotionController> _motion_controller;

    bool _is_drawer_opening_in_progress = false;

    bool _triggered_closing_lock_after_opening = false;

    uint32_t _timestamp_stall_guard_triggered_in_ms = 0;
    uint32_t _timestamp_drawer_opened_in_ms = 0;

    /* FUNCTIONS */

    void init() const;

    void handle_drawer_idle_state();

    void handle_drawer_active_state();

    void start_next_e_drawer_task();

    void check_if_push_to_close_is_triggered();

    void check_if_drawer_is_pulled_out();

    void handle_drawer_just_opened() override;

    void handle_drawer_just_closed() override;

    void handle_drawer_lock_control() override;

    void handle_drawer_moving_in();

    void handle_drawer_moving_out();

    void handle_stall_guard_triggered();

    void debug_prints_moving_e_drawer();
  };
} // namespace drawer

#endif // DRAWER_ELECTRICAL_DRAWER_HPP
