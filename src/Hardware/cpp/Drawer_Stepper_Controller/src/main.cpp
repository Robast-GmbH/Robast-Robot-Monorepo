#include <TMCStepper.h>
#include <SpeedyStepper.h>

#define EN_PIN 13 // Enable
#define DIR_PIN 14 // Direction
#define STEP_PIN 12 // Step
#define STALL_PIN_X 27 // Teensy pin that diag pin is attached to
#define SERIAL_PORT Serial2 // HardwareSerial port for Teensy 4.0
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f // Match to your driver

// lower value of STALL_VALUE increases stall sensitivity
// diag pin pulsed HIGH when SG_RESULT falls below 2*STALL_VALUE
// must be in StealthChop Mode for stallguard to work
// Value of TCOOLTHRS must be greater than TSTEP & TPWMTHRS
#define STALL_VALUE 100 // [0..255]
int stepTime = 160;
bool startup = true; // set false after homing

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

bool shaftVal = false;
bool stalled_X = false;

void stallInterruptX(){ // flag set when motor stalls
//stalled_X = true;
Serial.println("stalled interrupt");
}

void setup() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  //pinMode(DIR_PIN, OUTPUT);
  // shaft direction controlled through uart: driver.shaft(true or false)
  pinMode(STALL_PIN_X, INPUT);

  Serial.begin(115200);
  SERIAL_PORT.begin(115200); // HW UART drivers

  driver.begin(); // SPI: Init CS pins and possible SW SPI pins
  driver.toff(4); // Enables driver in software, changed from 5
  driver.blank_time(24);
  //driver.rms_current(400); // Set motor RMS current
  driver.microsteps(16); // Set microsteps to 1/16th
  //driver.en_pwm_mode(true); // Toggle stealthChop on TMC2130/2160/5130/5160
  driver.en_spreadCycle(true); // Toggle spreadCycle on TMC2208/2209/2224
  driver.pwm_autoscale(true); // Needed for stealthChop
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.shaft(shaftVal);
  // TCOOLTHRS needs to be set for stallgaurd to work //
  driver.TCOOLTHRS(0xFFFFF); // 20bit max
  driver.SGTHRS(STALL_VALUE);
  attachInterrupt(digitalPinToInterrupt(STALL_PIN_X), stallInterruptX, RISING);
  digitalWrite(EN_PIN, LOW); // Enable driver in hardware
}

int position = 0;

void makeSteps(int count,int delay){
  for(int i = 0; i<count; i++){
    digitalWrite(STEP_PIN,HIGH);
    delayMicroseconds(delay);
    digitalWrite(STEP_PIN,LOW);
    delayMicroseconds(delay);
    }
}

void motor(int steps, int stepDelay){
  digitalWrite(EN_PIN, LOW);
  driver.shaft(shaftVal);

  for(int i = 0; i<steps; i++){
    digitalWrite(STEP_PIN,HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(STEP_PIN,LOW);
    delayMicroseconds(stepDelay);
    if(shaftVal) position++;
    else position--;
  if(stalled_X){
    i = steps;
  }
 
  }  

  digitalWrite(EN_PIN,HIGH);
}

void homeX(){
  int homeDelay = 400;
  int steps = 10000;
  
  if(position<steps){
    Serial.println("Ausfahren");

    shaftVal = true;
    motor(steps-position,homeDelay);
  }
  delay(2000);
  if(position==steps&&!stalled_X){
    Serial.println("Einfahren");
    shaftVal=false;
    motor(steps,homeDelay);
  }
  delay(2000);
}

void loop() {
if(startup){ // home on starting up
startup = false;

}
homeX();
if(Serial.available()>0){
char readVal = Serial.read();
if (readVal == 'x'){
int steps = Serial.parseInt();
if(steps < 0){
shaftVal = true;
steps *= -1;
} else { shaftVal = false; }
motor(steps,160);
} else if (readVal == 'h'){
  stalled_X=false;
homeX();
}
}
}


