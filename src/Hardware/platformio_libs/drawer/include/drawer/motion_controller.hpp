#ifndef DRAWER_SPEED_CONTROLLER_HPP
#define DRAWER_SPEED_CONTROLLER_HPP

#include <memory>
#include <optional>

#include "drawer/electrical_drawer_config.hpp"
#include "interfaces/i_gpio_wrapper.hpp"
#include "motor/encoder.hpp"
#include "motor/encoder_monitor.hpp"
#include "motor/enconder_config.hpp"
#include "motor/motor.hpp"
#include "motor/motor_config.hpp"
#include "motor/motor_monitor.hpp"
#include "utils/e_drawer_task.hpp"

namespace drawer
{
  constexpr uint8_t MAX_SPEED_UINT8 = 255;
  constexpr uint8_t STALL_GUARD_DISABLED = 0;
  constexpr uint8_t TARGET_SPEED_ZERO = 0;
  constexpr uint8_t DRAWER_TARGET_HOMING_POSITION = 0;
  constexpr uint8_t DRAWER_HOMING_POSITION = 0;

  constexpr bool CONFIRM_MOTOR_CONTROL_CHANGE = true;
  constexpr bool LOCK_SWITCH_IS_NOT_PUSHED = false;
  constexpr bool PUSH_TO_CLOSE_NOT_TRIGGERED = false;
  constexpr bool MOTOR_IS_STALLED = true;
  constexpr bool MOTOR_IS_NOT_STALLED = false;

  class MotionController
  {
  public:
    MotionController(const uint32_t module_id,
                     const uint8_t id,
                     const bool use_encoder,
                     const uint8_t encoder_pin_a,
                     const uint8_t encoder_pin_b,
                     const uint8_t motor_driver_address,
                     const std::shared_ptr<motor::EncoderConfig> encoder_config,
                     const std::shared_ptr<motor::MotorConfig> motor_config,
                     const std::shared_ptr<motor::MotorMonitorConfig> motor_monitor_config,
                     const stepper_motor::StepperPinIdConfig &stepper_pin_id_config,
                     const std::shared_ptr<interfaces::IGpioWrapper> gpio_wrapper,
                     const std::shared_ptr<ElectricalDrawerConfig> e_drawer_config);

    void set_motor_driver_state(const bool enabled, const uint8_t motor_id) const;

    void set_target_speed_and_direction(const uint8_t target_speed, const bool use_acceleration_ramp);

    void set_target_speed_with_decelerating_ramp(const uint8_t target_speed);

    void handle_finished_drawer_homing();

    bool was_drawer_homed_once() const;

    bool is_drawer_moving_out() const;

    void reset_encoder();

    bool is_stall_guard_triggered();

    void handle_decelerating_for_moving_out_drawer();

    bool handle_finished_moving_out_drawer();

    void handle_decelerating_for_moving_in_drawer();

    bool handle_finished_moving_in_drawer();

    void set_is_idling(const bool is_idling);

    bool is_idling() const;

    uint8_t get_normed_target_speed_uint8(const uint32_t target_speed) const;

    uint8_t get_target_speed() const;

    uint8_t get_current_position() const;

    uint8_t get_normed_current_position() const;

    bool is_drawer_moving_in() const;

    bool is_push_to_close_is_triggered() const;

    void handle_stall_guard_triggered();

    void handle_motor_control() const;

    void update_position() const;

    void start_e_drawer_task(const utils::EDrawerTask &e_drawer_task);

    bool is_task_redundant(const uint8_t new_target_position) const;

    bool is_drawer_pulled_out() const;

    bool is_drawer_opening_in_progress() const;

  private:
    const uint32_t _module_id;
    const uint8_t _id;

    const std::shared_ptr<ElectricalDrawerConfig> _e_drawer_config;
    const std::shared_ptr<interfaces::IGpioWrapper> _gpio_wrapper;

    const std::shared_ptr<motor::Encoder> _encoder;
    const std::unique_ptr<motor::EncoderMonitor> _encoder_monitor;

    const std::shared_ptr<stepper_motor::Motor> _motor;
    const std::unique_ptr<motor::MotorMonitor> _motor_monitor;

    bool _is_idling = true;

    bool _is_drawer_opening_in_progress = false;

    bool _drawer_was_homed_once = false;
    bool _is_drawer_moving_out;

    bool _is_motor_monitor_stall_guard_triggered = false;
    bool _is_tmc_stall_guard_triggered = false;

    bool _triggered_deceleration_for_drawer_moving_out = false;
    bool _triggered_deceleration_for_drawer_moving_in = false;

    uint8_t _target_position_uint8 = 0;

    uint32_t _timestamp_movement_started_in_ms = 0;
    uint32_t _timestamp_movement_finished_in_ms = 0;
    uint32_t _timestamp_stall_guard_triggered_in_ms = 0;

    uint32_t get_normed_target_speed_uint32(const uint8_t target_speed) const;

    uint8_t get_scaled_moving_out_deceleration_distance() const;

    void start_homing_movement(const uint8_t target_speed);

    void start_normal_drawer_movement(const uint8_t target_speed, const bool use_acceleration_ramp);
  };

} // namespace drawer

#endif // DRAWER_SPEED_CONTROLLER_HPP