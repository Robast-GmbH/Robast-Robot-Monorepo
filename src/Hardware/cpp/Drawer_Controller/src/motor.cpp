#include "motor.hpp"

// higher value of STALL_VALUE increases stall sensitivity
// diag pin pulsed HIGH when SG_RESULT falls below 2*STALL_VALUE
// must be in StealthChop Mode for stallguard to work
// Value of TCOOLTHRS must be greater than TSTEP & TPWMTHRS
#define STALL_VALUE 50   // [0..255]
#define TOFF_VALUE  2

bool Motor::_is_stalled = false;

Motor::Motor(TMC2209Stepper* driver,
             std::shared_ptr<drawer_controller::IGpioWrapper> gpio_wrapper,
             const StepperPinIdConfig& stepper_pin_id_config)
{
  _driver = driver;
  _gpio_wrapper = gpio_wrapper;

  _stepper_en_tmc2209_pin_id = stepper_pin_id_config.stepper_en_tmc2209_pin_id;
  _stepper_stdby_tmc2209_pin_id = stepper_pin_id_config.stepper_stdby_tmc2209_pin_id;
  _stepper_spread_pin_id = stepper_pin_id_config.stepper_spread_pin_id;
  _stepper_dir_pin_id = stepper_pin_id_config.stepper_dir_pin_id;
  _stepper_diag_pin_id = stepper_pin_id_config.stepper_diag_pin_id;
  _stepper_index_pin_id = stepper_pin_id_config.stepper_index_pin_id;
  _stepper_step_pin_id = stepper_pin_id_config.stepper_step_pin_id;
}

void Motor::init()
{
  SERIAL_PORT.begin(115200);

  _driver->begin();

  // Sets the slow decay time (off time) [1... 15]. This setting also limits
  // the maximum chopper frequency. For operation with StealthChop,
  // this parameter is not used, but it is required to enable the motor.
  // In case of operation with StealthChop only, any setting is OK.
  _driver->toff(TOFF_VALUE);

  // VACTUAL allows moving the motor by UART control.
  // It gives the motor velocity in +-(2^23)-1 [μsteps / t]
  // 0: Normal operation. Driver reacts to STEP input.
  // /=0: Motor moves with the velocity given by VACTUAL.
  // Step pulses can be monitored via INDEX output.
  // The motor direction is controlled by the sign of VACTUAL.
  _driver->VACTUAL(_speed);

  // Comparator blank time. This time needs to safely cover the switching
  // event and the duration of the ringing on the sense resistor. For most
  // applications, a setting of 16 or 24 is good. For highly capacitive
  // loads, a setting of 32 or 40 will be required.
  _driver->blank_time(24);

  _driver->rms_current(400);   // mA
  _driver->microsteps(16);

  // Lower threshold velocity for switching on smart energy CoolStep and StallGuard to DIAG output
  _driver->TCOOLTHRS(0xFFFFF);   // 20bit max

  // CoolStep lower threshold [0... 15].
  // If SG_RESULT goes below this threshold, CoolStep increases the current to both coils.
  // 0: disable CoolStep
  _driver->semin(5);

  // CoolStep upper threshold [0... 15].
  // If SG is sampled equal to or above this threshold enough times,
  // CoolStep decreases the current to both coils.
  _driver->semax(2);

  // Sets the number of StallGuard2 readings above the upper threshold necessary
  // for each current decrement of the motor current.
  _driver->sedn(0b01);

  // StallGuard4 threshold [0... 255] level for stall detection. It compensates for
  // motor specific characteristics and controls sensitivity. A higher value gives a higher
  // sensitivity. A higher value makes StallGuard4 more sensitive and requires less torque to
  // indicate a stall. The double of this value is compared to SG_RESULT.
  // The stall output becomes active if SG_RESULT fall below this value.
  _driver->SGTHRS(STALL_VALUE);

  _driver->shaft(directionToShaftBool());

  Serial.print("\nTesting connection...");
  uint8_t result = _driver->test_connection();

  if (result)
  {
    Serial.println("failed!");
    Serial.print("Likely cause: ");

    switch (result)
    {
      case 1:
        Serial.println("loose connection");
        break;
      case 2:
        Serial.println("no power");
        break;
    }

    Serial.println("Fix the problem and reset board.");

    // We need this delay or messages above don't get fully printed out
    delay(100);
    abort();
  }

  // setStallGuard(true);

  Serial.println("Stepper initialized");
}

void Motor::stallISR()
{
  _is_stalled = true;
}

bool Motor::getIsStalled()
{
  return _is_stalled;
}

void Motor::setStallGuard(bool enable)
{
  if (_is_stall_guard_enabled && !enable)
  {
    detachInterrupt(STEPPER_DIAG_PIN);   // TODO@Jacob: Use interrup from port expander
    _is_stall_guard_enabled = false;
  }
  else if (enable)
  {
    attachInterrupt(STEPPER_DIAG_PIN, Motor::stallISR, RISING);   // TODO@Jacob: Use interrup from port expander
  }
}

void Motor::resetStallGuard()
{
  setSpeed(0, 0);
  _driver->VACTUAL(_speed);
  detachInterrupt(STEPPER_DIAG_PIN);   // TODO@Jacob: Use interrup from port expander
  _gpio_wrapper->digital_write(_stepper_en_tmc2209_pin_id, HIGH);
  delay(100);
  _gpio_wrapper->digital_write(_stepper_en_tmc2209_pin_id, LOW);
  delay(500);
  attachInterrupt(STEPPER_DIAG_PIN, stallISR, RISING);
  _is_stalled = false;
}

void Motor::setSpeed(int speed, int accelarationTime)
{
  this->_speed = speed;

  _driver->VACTUAL(this->_speed);
}

int Motor::getSpeed()
{
  return this->_speed;
}

bool Motor::directionToShaftBool()
{
  // TODO(andreas): Check if it's correct.
  if (_shaft_direction == clockwise)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void Motor::setDirection(Direction direction)
{
  // detachInterrupt(STEPPER_DIAG_PIN);
  _shaft_direction = direction;
  bool shaftDirectionAsBool = directionToShaftBool();
  _driver->shaft(shaftDirectionAsBool);
  // delay(500);
  // attachInterrupt(STEPPER_DIAG_PIN, stallISR, RISING);
}

Direction Motor::getDirection()
{
  return _shaft_direction;
}

void Motor::printStatus()
{
  Serial.print("Status: ");
  Serial.print(_driver->SG_RESULT(), DEC);
  Serial.print(" ");
  Serial.print(_driver->SG_RESULT() < STALL_VALUE, DEC);
  Serial.print(" ");
  byte drawer_diag_state;
  _gpio_wrapper->digital_read(_stepper_diag_pin_id, drawer_diag_state);
  Serial.print(drawer_diag_state);
  Serial.print(" ");
  Serial.println(_driver->cs2rms(_driver->cs_actual()), DEC);
}