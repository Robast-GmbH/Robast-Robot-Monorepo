#if !defined(DRAWER_CONTROLLER_MOTOR_HPP)
#define DRAWER_CONTROLLER_MOTOR_HPP

#include <TMCStepper.h>

#include <memory>

#include "i_gpio_wrapper.hpp"
#include "pinout_defines.h"

#define SERIAL_PORT Serial2
#define R_SENSE     0.33f   // Match to your driver

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
  };

  class Motor
  {
   public:
    Motor(uint8_t driver_address,
          std::shared_ptr<drawer_controller::IGpioWrapper> gpio_wrapper,
          const StepperPinIdConfig& stepper_pin_id_config);
    void init();
    void set_active_speed(uint32_t speed);
    void set_target_speed(uint32_t target_speed);
    void set_direction(Direction direction);
    void set_stall_guard(bool enable);
    void reset_stall_guard();

    uint32_t get_active_speed();
    uint32_t get_target_speed();
    bool get_is_stalled();
    Direction get_direction();
    void print_status();

   private:
    std::unique_ptr<TMC2209Stepper> _driver;

    std::shared_ptr<drawer_controller::IGpioWrapper> _gpio_wrapper;

    uint8_t _stepper_enn_tmc2209_pin_id;
    uint8_t _stepper_stdby_tmc2209_pin_id;
    uint8_t _stepper_spread_pin_id;
    uint8_t _stepper_dir_pin_id;
    uint8_t _stepper_diag_pin_id;
    uint8_t _stepper_index_pin_id;
    uint8_t _stepper_step_pin_id;

    uint32_t _active_speed = 0;
    uint32_t _target_speed = 0;

    static bool _is_stalled;
    bool _is_stall_guard_enabled = false;
    Direction _shaft_direction = clockwise;

    static void stall_ISR();
    bool direction_to_shaft_bool();
  };
}   // namespace stepper_motor

#endif   // DRAWER_CONTROLLER_MOTOR_HPP
