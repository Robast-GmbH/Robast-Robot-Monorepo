#ifndef MOTOR
#define MOTOR

#include <TMCStepper.h>
#include "pinout_defines.h"

#define SERIAL_PORT Serial2 // HardwareSerial port for Teensy 4.0
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f // Match to your driver


enum Direction{clockwise,counter_clockwise};

class Motor{
    private:
        TMC2209Stepper* driver;

        uint8_t stepper_diag_pin_;
        uint8_t stepper_enable_pin_;
        int32_t speed = 0;

        static bool isStalled;
        bool isStallGuardEnabled = false;
        Direction shaftDirection = clockwise;

        static void stallISR();
        bool directionToShaftBool();
    public:
        void init(TMC2209Stepper*,uint8_t,uint8_t);

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
