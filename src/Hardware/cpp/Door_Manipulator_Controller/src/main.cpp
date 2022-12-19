#include "SPI.h"
#include "norelem_stepper.hpp"
// #include "can.hpp"
// #include "can/can_db.hpp"
// #include "can/can_helper.h"

/*********************************************************************************************************
  GLOBAL VARIABLES AND CONSTANTS
*********************************************************************************************************/

#define DRAWER_CONTROLLER_ID 1 //TODO: Every DRAWER_CONTROLLER needs to have his own id

norelem_stepper::NorelemStepper STEPPER_1 = norelem_stepper::NorelemStepper();
norelem_stepper::NorelemStepper STEPPER_2 = norelem_stepper::NorelemStepper();

// can::Can CAN = can::Can(DRAWER_CONTROLLER_ID, &LOCK_1, &LOCK_2);

unsigned long last_timestamp = 0;
#define CLOCK_RATE_MOTOR_START_PIN 2000 //ms
bool motor_clock_state;

// Fahrprogramme
// E1 = Fahre Relativ -20 Schritte
// E2 = Fahre Relativ +20 Schritte
// E1 E2 = Geschwindigkeit 0 Hz
// E3 = Geschwindigkeit -50 Hz
// E3 E1 = Geschwindigkeit +50 Hz
// E3 E2 = Geschwindigkeit -100 Hz
// E3 E2 E1 = Geschwindigkeit +100 Hz


/*********************************************************************************************************
  SETUP
*********************************************************************************************************/

void setup()
{
  Serial.begin(115200);

  // CAN.initialize_can_controller();

  STEPPER_1.initialize(MOTOR_1_START_PIN, MOTOR_1_INPUT_1_PIN, MOTOR_1_INPUT_2_PIN, MOTOR_1_INPUT_3_PIN);
  STEPPER_2.initialize(MOTOR_2_START_PIN, MOTOR_2_INPUT_1_PIN, MOTOR_2_INPUT_2_PIN, MOTOR_2_INPUT_3_PIN);
}


/*********************************************************************************************************
  LOOP
*********************************************************************************************************/

void loop()
{
  unsigned long current_timestamp = millis();

  if ((current_timestamp - last_timestamp) > (CLOCK_RATE_MOTOR_START_PIN / 2))
  {
    if (motor_clock_state)
    {
      STEPPER_1.set_motor_start_pin(true);
      STEPPER_2.set_motor_start_pin(true);
      motor_clock_state = false;
    }
    else
    {
      STEPPER_1.set_motor_start_pin(false);
      STEPPER_2.set_motor_start_pin(false);
      motor_clock_state = true;
    }    
    last_timestamp = current_timestamp;
  }

  // CAN.handle_receiving_can_msg();

  // led_strip::handle_led_control();

  // CAN.handle_sending_drawer_status_feedback();
}


