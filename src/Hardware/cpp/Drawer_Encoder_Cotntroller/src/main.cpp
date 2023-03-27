/*
 *  Application note: incremental encoder and RS422/RS485 Shield  
 *  Version 1.0
 *  Copyright (C) 2020  Hartmut Wendt  www.zihatec.de
  *  
 *  used encoder: SICK DFS60 www.sick.com
 *  
 *
 *  This program  is free software: you can redistribute it and/or modify
 *  it under the terms  of the GNU General Public License as published by
 *  the Free Software Foundation,  either version 3 of the License, or
 *  (at your option) any later version.
  *
 *  This program is distributed in the hope that it will be useful,
 *  but  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY  or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License  for more details.
 *
 *  You should have received a copy of the GNU General  Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/   

#include <Arduino.h>
// Pin definitions

const byte COSPin = 25;   // RS422 input for A channel  (cos signal) 
const byte SINPin = 26;   // RS422 input for B channel (sin signal)  

// variables
enum {ENC_STOP, ENC_CLOCKWISE_ROTATION, ENC_COUNTERCLOCKWISE_ROTATION};  // encoder operation modes
volatile byte encoder_state = ENC_STOP;
volatile  int encoder_position = 0; 
volatile int encoder_oldpos = 0; 

void encoder_isr() {
  
  if  (digitalRead(SINPin)  == LOW) {
    // clockwise rotation
    encoder_state=ENC_CLOCKWISE_ROTATION;
    encoder_position++;
  } else {
    //counter-clockwise rotation
    encoder_state=ENC_COUNTERCLOCKWISE_ROTATION;
    encoder_position--;    
  } 
}

void  setup() {
  Serial.begin(115200); //Use serial monitor for debugging

  // Define pins for input and output
  pinMode(SINPin, INPUT);

  // set  internal pullup resistor for interrupt pin
  pinMode(COSPin, INPUT_PULLUP);
  
  
  // set 1st interrupt service  routine to COSPin and 'RISING' edge 
  attachInterrupt(digitalPinToInterrupt(COSPin),  encoder_isr, RISING);


}


void loop() {
  // Detect Encoder Stop
  if (encoder_oldpos  == encoder_position) encoder_state= ENC_STOP;

  // output encoder incremental  and status
  Serial.print("Encoder position: ");
  Serial.print(encoder_position);
  Serial.print(", Encoder state: ");
  
  if (encoder_state==ENC_CLOCKWISE_ROTATION)  {
    Serial.println("Clockwise Rotation");
        
  } else if (encoder_state==ENC_COUNTERCLOCKWISE_ROTATION)  {
    Serial.println("Counter-Clockwise Rotation");
    
  } else {
    Serial.println("Stop");  
  }
  
  encoder_oldpos = encoder_position;
  
  delay(500);
}


