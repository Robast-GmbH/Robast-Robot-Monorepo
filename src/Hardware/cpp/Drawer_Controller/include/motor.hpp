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
    void set_speed(int speed, int accelarationTime);
    void setDirection(Direction direction);
    void setStallGuard(bool enable);
    void resetStallGuard();

    int get_speed();
    bool getIsStalled();
    Direction get_direction();
    void printStatus();

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

    int32_t _speed = 0;

    static bool _is_stalled;
    bool _is_stall_guard_enabled = false;
    Direction _shaft_direction = clockwise;

    static void stallISR();
    bool directionToShaftBool();
  };
}   // namespace stepper_motor

#endif   // DRAWER_CONTROLLER_MOTOR_HPP
