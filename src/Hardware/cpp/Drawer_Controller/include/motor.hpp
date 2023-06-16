#ifndef MOTOR
#define MOTOR

#include <TMCStepper.h>

#include <memory>

#include "i_gpio_wrapper.hpp"
#include "pinout_defines.h"

#define SERIAL_PORT    Serial2   // HardwareSerial port for Teensy 4.0
#define DRIVER_ADDRESS 0b00      // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE        0.11f     // Match to your driver

enum Direction
{
  clockwise,
  counter_clockwise
};

struct StepperPinIdConfig
{
  uint8_t stepper_en_tmc2209_pin_id;
  uint8_t stepper_stdby_tmc2209_pin_id;
  uint8_t stepper_spread_pin_id;
  uint8_t stepper_dir_pin_id;
  uint8_t stepper_diag_pin_id;
  uint8_t stepper_index_pin_id;
  uint8_t stepper_step_pin_id;
};

class Motor
{
 private:
  TMC2209Stepper* _driver;

  std::shared_ptr<drawer_controller::IGpioWrapper> _gpio_wrapper;

  uint8_t _stepper_en_tmc2209_pin_id;
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

 public:
  Motor(TMC2209Stepper* driver,
        std::shared_ptr<drawer_controller::IGpioWrapper> gpio_wrapper,
        const StepperPinIdConfig& stepper_pin_id_config);
  void init();
  void setSpeed(int speed, int accelarationTime);
  void setDirection(Direction direction);
  void setStallGuard(bool enable);
  void resetStallGuard();

  int getSpeed();
  bool getIsStalled();
  Direction getDirection();
  void printStatus();
};

#endif
