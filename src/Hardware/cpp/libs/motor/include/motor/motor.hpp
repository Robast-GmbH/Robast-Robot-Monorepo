#ifndef DRAWER_CONTROLLER_MOTOR_HPP
#define DRAWER_CONTROLLER_MOTOR_HPP

#include <TMCStepper.h>

#include <memory>

#include "debug/debug.hpp"
#include "interfaces/i_gpio_wrapper.hpp"
#include "switch/switch.hpp"

// higher value of STALL_VALUE increases stall sensitivity
// diag pin pulsed HIGH when SG_RESULT falls below 2*STALL_VALUE
// must be in StealthChop Mode for stallguard to work
// Value of TCOOLTHRS must be greater than TSTEP & TPWMTHRS
#define STALL_DEFAULT_VALUE        50   // [0..255]
#define TOFF_VALUE                 2
#define SERIAL_PORT                Serial2
#define R_SENSE                    0.33f   // Match to your driver
#define DEFAULT_MOTOR_ACCELERATION 10
#define INSTANT_ACCELERATION       0

#define STALL_GUARD_READER_THRESHOLD           0.9
#define STALL_GUARD_READER_WEIGHT_NEW_READINGS 1.0

#define TCOOLTHRS_VALUE 120   // TODO: Make this configurable via CAN
#define TPWMTHRS_VALUE  10    // TODO: Make this configurable via CAN

namespace stepper_motor
{

  enum Direction
  {
    clockwise,
    counter_clockwise
  };

  struct StepperPinIdConfig
  {
    uint8_t stepper_enn_tmc2209_pin_id;
    uint8_t stepper_stdby_tmc2209_pin_id;
    uint8_t stepper_spread_pin_id;
    uint8_t stepper_dir_pin_id;
    uint8_t stepper_diag_pin_id;
    uint8_t stepper_index_pin_id;
    uint8_t stepper_step_pin_id;
    uint8_t port_expander_ninterrupt_pin_id;
  };

  class Motor
  {
   public:
    Motor(const uint8_t driver_address,
          const std::shared_ptr<drawer_controller::IGpioWrapper> gpio_wrapper,
          const StepperPinIdConfig &stepper_pin_id_config,
          const bool shaft_direction_is_inverted);

    void init();

    void set_target_speed_with_decelerating_ramp(uint32_t target_speed,
                                                 int32_t ramp_distance_int32,
                                                 int32_t starting_position_int32);

    void set_target_speed_with_accelerating_ramp(uint32_t target_speed, int16_t acceleration);

    void set_target_speed_instantly(uint32_t target_speed);

    void set_direction(Direction direction);

    void set_stall_guard(uint8_t stall_guard_value);

    void handle_motor_control(int32_t current_position_int32);

    uint32_t get_active_speed() const;

    uint32_t get_target_speed() const;

    bool get_is_stalled() const;

    Direction get_direction() const;

    void print_status();

    void reset_stall_guard();

   private:
    const std::unique_ptr<TMC2209Stepper> _driver;

    bool _driver_is_enabled;

    const std::shared_ptr<drawer_controller::IGpioWrapper> _gpio_wrapper;

    const uint8_t _stepper_enn_tmc2209_pin_id;
    const uint8_t _stepper_stdby_tmc2209_pin_id;
    const uint8_t _stepper_spread_pin_id;
    const uint8_t _stepper_dir_pin_id;
    const uint8_t _stepper_diag_pin_id;
    const uint8_t _stepper_index_pin_id;
    const uint8_t _stepper_step_pin_id;
    const uint8_t _port_expander_ninterrupt_pin_id;

    uint32_t _active_speed = 0;
    uint32_t _target_speed = 0;
    uint16_t _acceleration;
    uint32_t _starting_speed_before_ramp_uint32;
    uint32_t _ramp_distance_uint32;
    int32_t _starting_position_int32;
    bool _speed_ramp_in_progress = false;

    uint32_t _start_ramp_timestamp;

    bool _is_stalled;
    const std::unique_ptr<drawer_controller::Switch> _stall_guard_reader;
    uint8_t _current_stall_guard_value;

    const bool _shaft_direction_is_inverted;
    Direction _shaft_direction;

    bool direction_to_shaft_bool(Direction direction);

    void set_active_speed(uint32_t speed);

    void handle_time_dependent_acceleration();

    void handle_position_dependent_deceleration(int32_t current_position_int32);

    void finish_speed_ramp(uint32_t final_speed);

    uint32_t get_delta_speed(int32_t current_position_int32) const;

    uint32_t get_dt_since_start_in_ms() const;

    uint32_t calculate_new_active_speed(int32_t current_position_int32) const;

    void read_stall_guard_pin();

    void setup_driver();

    void enable_driver();

    void disable_driver();
  };
}   // namespace stepper_motor

#endif   // DRAWER_CONTROLLER_MOTOR_HPP
