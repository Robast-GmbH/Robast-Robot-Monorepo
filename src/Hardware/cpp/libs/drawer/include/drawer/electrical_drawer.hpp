#ifndef DRAWER_ELECTRICAL_DRAWER_HPP
#define DRAWER_ELECTRICAL_DRAWER_HPP

#include <Arduino.h>

#include <memory>
#include <optional>

#include "can_toolbox/can_utils.hpp"
#include "drawer/electrical_drawer_config.hpp"
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
  constexpr uint8_t MAX_SPEED_UINT8 = 255;
  constexpr uint8_t DRAWER_TARGET_HOMING_POSITION = 0;
  constexpr uint8_t DRAWER_HOMING_POSITION = 0;
  constexpr uint8_t STALL_GUARD_DISABLED = 0;
  constexpr uint8_t TARGET_SPEED_ZERO = 0;
  constexpr bool IS_HOMING = true;
  constexpr bool IS_NOT_HOMING = false;
  constexpr bool DO_NOT_USE_ACCELERATION_RAMP = false;
  constexpr bool PUSH_TO_CLOSE_TRIGGERED = true;
  constexpr bool PUSH_TO_CLOSE_NOT_TRIGGERED = false;
  constexpr bool MOTOR_IS_STALLED = true;
  constexpr bool MOTOR_IS_NOT_STALLED = false;

  class ElectricalDrawer : public interfaces::IDrawer
  {
   public:
    ElectricalDrawer(const uint32_t module_id,
                     const uint8_t id,
                     const std::shared_ptr<robast_can_msgs::CanDb> can_db,
                     const std::shared_ptr<interfaces::IGpioWrapper> gpio_wrapper,
                     const stepper_motor::StepperPinIdConfig &stepper_pin_id_config,
                     const bool use_encoder,
                     const uint8_t encoder_pin_a,
                     const uint8_t encoder_pin_b,
                     const uint8_t motor_driver_address,
                     const std::shared_ptr<motor::MotorConfig> motor_config,
                     const std::shared_ptr<switch_lib::Switch> endstop_switch,
                     const std::optional<std::shared_ptr<lock::ElectricalDrawerLock>> e_drawer_lock,
                     const std::shared_ptr<ElectricalDrawerConfig> e_drawer_config,
                     const std::shared_ptr<motor::EncoderConfig> encoder_config,
                     const std::shared_ptr<motor::MotorMonitorConfig> motor_monitor_config);

    std::optional<robast_can_msgs::CanMessage> can_out() override;

    void update_state() override;

    void unlock();

    void add_e_drawer_task_to_queue(const utils::EDrawerTask &e_drawer_task);

   private:
    const uint32_t _module_id;
    const uint8_t _id;

    const std::shared_ptr<interfaces::IGpioWrapper> _gpio_wrapper;

    const stepper_motor::StepperPinIdConfig _stepper_pin_id_config;

    const std::shared_ptr<motor::Encoder> _encoder;

    const std::shared_ptr<switch_lib::Switch> _endstop_switch;

    // optional because the lock is not always installed (e.g. in the partial drawer)
    const std::optional<std::shared_ptr<lock::ElectricalDrawerLock>> _e_drawer_lock;

    const std::unique_ptr<can_toolbox::CanUtils> _can_utils;

    const std::shared_ptr<stepper_motor::Motor> _motor;

    std::unique_ptr<utils::Queue<utils::EDrawerTask>> _e_drawer_task_queue =
      std::make_unique<utils::Queue<utils::EDrawerTask>>();

    const std::shared_ptr<ElectricalDrawerConfig> _config;

    const std::unique_ptr<motor::EncoderMonitor> _encoder_monitor;

    const std::unique_ptr<motor::MotorMonitor> _motor_monitor;

    bool _drawer_was_homed_once = false;

    bool _is_drawer_moving_out;
    bool _triggered_deceleration_for_drawer_moving_out = false;
    bool _is_idling = true;
    bool _triggered_deceleration_for_drawer_moving_in = false;

    uint8_t _target_position_uint8 = 0;

    bool _triggered_closing_lock_after_opening = false;

    uint32_t _timestamp_stall_guard_triggered_in_ms = 0;
    uint32_t _timestamp_movement_started_in_ms = 0;

    bool _is_motor_monitor_stall_guard_triggered = false;
    bool _is_tmc_stall_guard_triggered = false;

    /* FUNCTIONS */

    void init() const;

    void handle_drawer_idle_state();

    void handle_drawer_active_state();

    void start_next_e_drawer_task();

    void start_normal_drawer_movement(const uint8_t target_speed, const bool use_acceleration_ramp);

    void start_homing_movement(const uint8_t target_speed);

    bool check_and_handle_initial_drawer_homing();

    void handle_drawer_just_opened() override;

    void handle_drawer_just_closed() override;

    void handle_e_drawer_lock_control() override;

    void handle_drawer_moving_in();

    void handle_drawer_moving_out();

    void handle_decelerating_for_moving_in_drawer();

    void handle_decelerating_for_moving_out_drawer();

    void handle_finished_moving_in_drawer();

    void handle_finished_moving_out_drawer();

    bool is_stall_guard_triggered();

    void handle_stall_guard_triggered();

    void set_target_speed_and_direction(const uint8_t target_speed, const bool use_acceleration_ramp);

    void reset_encoder_if_endstop_is_pushed();

    void debug_prints_moving_e_drawer();

    uint32_t get_normed_target_speed_uint32(const uint8_t target_speed) const;

    uint8_t get_normed_target_speed_uint8(const uint32_t target_speed) const;

    bool is_stall_guard_triggered() const;
  };
}   // namespace drawer

#endif   // DRAWER_ELECTRICAL_DRAWER_HPP
