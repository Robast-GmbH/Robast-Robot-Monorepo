#include "SPI.h"
#include "norelem_stepper.hpp"
#include "can.hpp"
#include "can/can_db.hpp"
#include "can/can_helper.h"

/*********************************************************************************************************
  GLOBAL VARIABLES AND CONSTANTS
*********************************************************************************************************/

#define DRAWER_CONTROLLER_ID 6 //TODO: Every DRAWER_CONTROLLER needs to have his own id

norelem_stepper::NorelemStepper STEPPER_1 = norelem_stepper::NorelemStepper();
norelem_stepper::NorelemStepper STEPPER_2 = norelem_stepper::NorelemStepper();
norelem_stepper::NorelemStepper STEPPER_3 = norelem_stepper::NorelemStepper();

can::Can CAN = can::Can(DRAWER_CONTROLLER_ID, &STEPPER_1, &STEPPER_2, &STEPPER_3);

unsigned long last_timestamp = 0;
#define CLOCK_RATE_MOTOR_START_PIN 2000 //ms
bool motor_clock_state;

// Fahrprogramme
// alle 0   = Geschwindigkeit    0 Hz
// E1       = Geschwindigkeit    0 Hz
// E2       = Geschwindigkeit  -50 Hz
// E1 E2    = Geschwindigkeit  +50 Hz
// E3       = Geschwindigkeit -100 Hz
// E3 E1    = Geschwindigkeit +100 Hz
// E3 E2    = Geschwindigkeit -150 Hz
// E3 E2 E1 = Geschwindigkeit +150 Hz


/*********************************************************************************************************
  SETUP
*********************************************************************************************************/

void setup()
{
  Serial.begin(115200);

  CAN.initialize_can_controller();

  STEPPER_1.initialize(MOTOR_1_START_PIN, MOTOR_1_INPUT_1_PIN, MOTOR_1_INPUT_2_PIN, MOTOR_1_INPUT_3_PIN);
  STEPPER_2.initialize(MOTOR_2_START_PIN, MOTOR_2_INPUT_1_PIN, MOTOR_2_INPUT_2_PIN, MOTOR_2_INPUT_3_PIN);
  STEPPER_3.initialize(MOTOR_3_START_PIN, MOTOR_3_INPUT_1_PIN, MOTOR_3_INPUT_2_PIN, MOTOR_3_INPUT_3_PIN);
}


/*********************************************************************************************************
  LOOP
*********************************************************************************************************/

void loop()
{
  CAN.handle_receiving_can_msg();
}


