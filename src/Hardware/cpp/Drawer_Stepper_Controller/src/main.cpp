#include <TMCStepper.h>
#include <SpeedyStepper.h>

#define EN_PIN 13 // Enable
#define DIR_PIN 14 // Direction
#define STEP_PIN 12 // Step
#define STALL_PIN_X 27 // Teensy pin that diag pin is attached to
#define SERIAL_PORT Serial2 // HardwareSerial port for Teensy 4.0
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f // Match to your driver

// higher value of STALL_VALUE increases stall sensitivity
// diag pin pulsed HIGH when SG_RESULT falls below 2*STALL_VALUE
// must be in StealthChop Mode for stallguard to work
// Value of TCOOLTHRS must be greater than TSTEP & TPWMTHRS
#define STALL_VALUE 100 // [0..255]
#define TOFF_VALUE 2
int stepTime = 160;


TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

int32_t speed = 5000;

bool isStalled = false;
bool shaftDirection = true;

void stallISR(){
 isStalled=true;
}

void setup() {
  pinMode(STALL_PIN_X,INPUT);
  Serial.begin(115200);         // Init serial port and set baudrate
  while(!Serial);               // Wait for serial port to connect
  Serial.println("\nStart...");

  SERIAL_PORT.begin(115200);
  
 
  driver.begin();

  // Sets the slow decay time (off time) [1... 15]. This setting also limits
  // the maximum chopper frequency. For operation with StealthChop,
  // this parameter is not used, but it is required to enable the motor.
  // In case of operation with StealthChop only, any setting is OK.
  driver.toff(TOFF_VALUE);

  // VACTUAL allows moving the motor by UART control.
  // It gives the motor velocity in +-(2^23)-1 [μsteps / t]
  // 0: Normal operation. Driver reacts to STEP input.
  // /=0: Motor moves with the velocity given by VACTUAL. 
  // Step pulses can be monitored via INDEX output.
  // The motor direction is controlled by the sign of VACTUAL.
  driver.VACTUAL(speed);

  // Comparator blank time. This time needs to safely cover the switching
  // event and the duration of the ringing on the sense resistor. For most
  // applications, a setting of 16 or 24 is good. For highly capacitive
  // loads, a setting of 32 or 40 will be required.
  driver.blank_time(24);

  driver.rms_current(400); // mA
  driver.microsteps(16);

  // Lower threshold velocity for switching on smart energy CoolStep and StallGuard to DIAG output
  driver.TCOOLTHRS(0xFFFFF); // 20bit max
  
  // CoolStep lower threshold [0... 15].
  // If SG_RESULT goes below this threshold, CoolStep increases the current to both coils.
  // 0: disable CoolStep
  driver.semin(5);

  // CoolStep upper threshold [0... 15].
  // If SG is sampled equal to or above this threshold enough times,
  // CoolStep decreases the current to both coils.
  driver.semax(2);

  // Sets the number of StallGuard2 readings above the upper threshold necessary
  // for each current decrement of the motor current.
  driver.sedn(0b01);

  // StallGuard4 threshold [0... 255] level for stall detection. It compensates for
  // motor specific characteristics and controls sensitivity. A higher value gives a higher
  // sensitivity. A higher value makes StallGuard4 more sensitive and requires less torque to
  // indicate a stall. The double of this value is compared to SG_RESULT.
  // The stall output becomes active if SG_RESULT fall below this value.
  driver.SGTHRS(STALL_VALUE);

  driver.shaft(shaftDirection);

  Serial.print("\nTesting connection...");
  uint8_t result = driver.test_connection();

  if (result) {
    Serial.println("failed!");
    Serial.print("Likely cause: ");

    switch(result) {
        case 1: Serial.println("loose connection"); break;
        case 2: Serial.println("no power"); break;
    }

    Serial.println("Fix the problem and reset board.");

    // We need this delay or messages above don't get fully printed out
    delay(100);
    abort();
  }

  Serial.println("OK");
  attachInterrupt(digitalPinToInterrupt(STALL_PIN_X), stallISR, RISING);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);
}

void loop() {
  static uint32_t last_time = 0;

  uint32_t ms = millis();

  while (Serial.available() > 0) {
    int8_t read_byte = Serial.read();

    if (read_byte == '0') {
      Serial.print("Motor ");

      if (driver.toff() == 0) {
        Serial.print("already ");
      }

      Serial.println("disabled.");
      driver.toff(0);
    } else if (read_byte == '1') {
      Serial.print("Motor ");

      if (driver.toff() != 0) {
        Serial.print("already ");
      }

      Serial.println("enabled.");
      driver.toff(TOFF_VALUE);
    } else if (read_byte == '+') {
      speed += 1000;

      if (speed == 0) {
        Serial.println("Hold motor.");
      } else {
        Serial.println("Increase speed.");
      }

      driver.VACTUAL(speed);
    } else if (read_byte == '-') {
      speed -= 1000;

      if (speed == 0) {
        Serial.println("Hold motor.");
      } else {
        Serial.println("Decrease speed.");
      }

      driver.VACTUAL(speed);
    } else if(read_byte =='2'){
      Serial.println("Reset Error.");
      detachInterrupt(STALL_PIN_X);
         
      digitalWrite(EN_PIN,HIGH);
      digitalWrite(EN_PIN,LOW);
      driver.VACTUAL(speed);
      delay(500);
      attachInterrupt(STALL_PIN_X,stallISR,RISING);
    }else if(read_byte =='3'){
      Serial.println("Toggle direction.");
       detachInterrupt(STALL_PIN_X);
      shaftDirection=!shaftDirection;
      driver.shaft(shaftDirection);
         delay(500);
      attachInterrupt(STALL_PIN_X,stallISR,RISING);
    }
  }

  if(isStalled){
    driver.VACTUAL(0);
    isStalled=false;
  }

  if((ms-last_time) > 100 && driver.toff() != 0 && speed != 0) { // run every 0.1s
    last_time = ms;

    Serial.print("Status: ");
    Serial.print(driver.SG_RESULT(), DEC);
    Serial.print(" ");
    Serial.print(driver.SG_RESULT() < STALL_VALUE, DEC);
     Serial.print(" ");
    Serial.print(digitalRead(STALL_PIN_X));
    Serial.print(" ");
    Serial.println(driver.cs2rms(driver.cs_actual()), DEC);
    
  }
}