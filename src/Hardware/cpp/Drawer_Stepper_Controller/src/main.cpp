#include "can.hpp"
#include "can/can_db.hpp"
#include "can/can_helper.h"
#include "pinout_defines.h"
#include "motor.hpp"

#define DRAWER_CONTROLLER_ID 1
TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, 0);

Motor motor = Motor(&driver); 

lock::Lock LOCK_1 = lock::Lock();
lock::Lock LOCK_2 = lock::Lock();

can::Can CAN = can::Can(DRAWER_CONTROLLER_ID, &LOCK_1, &LOCK_2,&motor);

void setup() {
  
  Serial.begin(115200);         // Init serial port and set baudrate
  while(!Serial);               // Wait for serial port to connect
  Serial.println("\nStart...");
  //motor.init();
  CAN.initialize_can_controller();
}

void loop() {
  static uint32_t last_time = 0;
  uint32_t ms = millis();

  while (Serial.available() > 0) {
    int8_t read_byte = Serial.read();
    int currentSpeed = motor.getSpeed();
    if (read_byte == '+') {
      
      Serial.println("Increase speed.");
      motor.setSpeed(currentSpeed+1000,0);

    } else if (read_byte == '-') {
    
      if (currentSpeed-1000 <= 0) {
        Serial.println("Hold motor.");
        motor.setSpeed(0,0);
      } else {
        Serial.println("Decrease speed.");
        motor.setSpeed(currentSpeed-1000,0);
      }
  
    } else if(read_byte =='0'){
      Serial.println("Reset Error.");
      motor.resetStallGuard();
    }else if(read_byte =='1'){
      Serial.println("Toggle direction.");
      Direction currentDirection = motor.getDirection();
      motor.setDirection(currentDirection==clockwise?counter_clockwise:clockwise);      
    }
  }

  if(motor.getIsStalled()){
    Serial.println("isStalled");
    motor.resetStallGuard();
  }


      CAN.handle_receiving_can_msg();
  
  if((ms-last_time) > 100 && motor.getSpeed() != 0) { // run every 0.1s
    last_time = ms;
    motor.printStatus();
  }
}