#include "lock.hpp"
#include "led_strip.hpp"
#include "can.hpp"
#include "can/can_db.hpp"
#include "can/can_helper.h"



/*********************************************************************************************************
  GLOBAL VARIABLES AND CONSTANTS
*********************************************************************************************************/

#define DRAWER_CONTROLLER_ID 2 //TODO: Every DRAWER_CONTROLLER needs to have his own id

lock::Lock LOCK_1 = lock::Lock();
lock::Lock LOCK_2 = lock::Lock();

can::Can CAN = can::Can(DRAWER_CONTROLLER_ID, LOCK_1, LOCK_2);


/*********************************************************************************************************
  SETUP
*********************************************************************************************************/

void setup()
{
  Serial.begin(115200);

  CAN.initialize_can_controller();

  LOCK_1.initialize_locks(PWR_OPEN_LOCK1_PIN, PWR_CLOSE_LOCK1_PIN, SENSOR_LOCK1_PIN, SENSOR_DRAWER1_CLOSED_PIN);
  LOCK_2.initialize_locks(PWR_OPEN_LOCK2_PIN, PWR_CLOSE_LOCK2_PIN, SENSOR_LOCK2_PIN, SENSOR_DRAWER2_CLOSED_PIN);

  led_strip::initialize_led_strip();
}


/*********************************************************************************************************
  LOOP
*********************************************************************************************************/

void loop()
{
  CAN.handle_receiving_can_msg();

  LOCK_1.handle_lock_control();
  // LOCK_2.handle_lock_control();

  led_strip::handle_led_control();
  
  LOCK_1.handle_reading_sensors();
  // LOCK_2.handle_reading_sensors();

  CAN.handle_sending_drawer_status_feedback();
}


