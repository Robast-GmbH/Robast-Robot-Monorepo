#include "motor.hpp"

// higher value of STALL_VALUE increases stall sensitivity
// diag pin pulsed HIGH when SG_RESULT falls below 2*STALL_VALUE
// must be in StealthChop Mode for stallguard to work
// Value of TCOOLTHRS must be greater than TSTEP & TPWMTHRS
#define STALL_VALUE 50   // [0..255]
#define TOFF_VALUE  2

bool Motor::isStalled = false;

Motor::Motor(TMC2209Stepper* driver)
{
  this->driver = driver;
}

void Motor::init()
{
  SERIAL_PORT.begin(115200);

  driver->begin();

  // Sets the slow decay time (off time) [1... 15]. This setting also limits
  // the maximum chopper frequency. For operation with StealthChop,
  // this parameter is not used, but it is required to enable the motor.
  // In case of operation with StealthChop only, any setting is OK.
  driver->toff(TOFF_VALUE);

  // VACTUAL allows moving the motor by UART control.
  // It gives the motor velocity in +-(2^23)-1 [μsteps / t]
  // 0: Normal operation. Driver reacts to STEP input.
  // /=0: Motor moves with the velocity given by VACTUAL.
  // Step pulses can be monitored via INDEX output.
  // The motor direction is controlled by the sign of VACTUAL.
  driver->VACTUAL(speed);

  // Comparator blank time. This time needs to safely cover the switching
  // event and the duration of the ringing on the sense resistor. For most
  // applications, a setting of 16 or 24 is good. For highly capacitive
  // loads, a setting of 32 or 40 will be required.
  driver->blank_time(24);

  driver->rms_current(400);   // mA
  driver->microsteps(16);

  // Lower threshold velocity for switching on smart energy CoolStep and StallGuard to DIAG output
  driver->TCOOLTHRS(0xFFFFF);   // 20bit max

  // CoolStep lower threshold [0... 15].
  // If SG_RESULT goes below this threshold, CoolStep increases the current to both coils.
  // 0: disable CoolStep
  driver->semin(5);

  // CoolStep upper threshold [0... 15].
  // If SG is sampled equal to or above this threshold enough times,
  // CoolStep decreases the current to both coils.
  driver->semax(2);

  // Sets the number of StallGuard2 readings above the upper threshold necessary
  // for each current decrement of the motor current.
  driver->sedn(0b01);

  // StallGuard4 threshold [0... 255] level for stall detection. It compensates for
  // motor specific characteristics and controls sensitivity. A higher value gives a higher
  // sensitivity. A higher value makes StallGuard4 more sensitive and requires less torque to
  // indicate a stall. The double of this value is compared to SG_RESULT.
  // The stall output becomes active if SG_RESULT fall below this value.
  driver->SGTHRS(STALL_VALUE);

  driver->shaft(directionToShaftBool());

  Serial.print("\nTesting connection...");
  uint8_t result = driver->test_connection();

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
  isStalled = true;
}

bool Motor::getIsStalled()
{
  return isStalled;
}

void Motor::setStallGuard(bool enable)
{
  if (isStallGuardEnabled && !enable)
  {
    detachInterrupt(STEPPER_DIAG_PIN);
    isStallGuardEnabled = false;
  }
  else if (enable)
  {
    attachInterrupt(STEPPER_DIAG_PIN, Motor::stallISR, RISING);
  }
}

void Motor::resetStallGuard()
{
  setSpeed(0, 0);
  driver->VACTUAL(speed);
  detachInterrupt(STEPPER_DIAG_PIN);
  digitalWrite(STEPPER_ENABLE_PIN, HIGH);
  delay(100);
  digitalWrite(STEPPER_ENABLE_PIN, LOW);
  delay(500);
  attachInterrupt(STEPPER_DIAG_PIN, stallISR, RISING);
  isStalled = false;
}

void Motor::setSpeed(int speed, int accelarationTime)
{
  this->speed = speed;

  driver->VACTUAL(this->speed);
}

int Motor::getSpeed()
{
  return this->speed;
}

bool Motor::directionToShaftBool()
{
  // TODO(andreas): Check if it's correct.
  if (shaftDirection == clockwise)
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
  shaftDirection = direction;
  bool shaftDirectionAsBool = directionToShaftBool();
  driver->shaft(shaftDirectionAsBool);
  // delay(500);
  // attachInterrupt(STEPPER_DIAG_PIN, stallISR, RISING);
}

Direction Motor::getDirection()
{
  return shaftDirection;
}

void Motor::printStatus()
{
  Serial.print("Status: ");
  Serial.print(driver->SG_RESULT(), DEC);
  Serial.print(" ");
  Serial.print(driver->SG_RESULT() < STALL_VALUE, DEC);
  Serial.print(" ");
  Serial.print(digitalRead(STEPPER_DIAG_PIN));
  Serial.print(" ");
  Serial.println(driver->cs2rms(driver->cs_actual()), DEC);
}