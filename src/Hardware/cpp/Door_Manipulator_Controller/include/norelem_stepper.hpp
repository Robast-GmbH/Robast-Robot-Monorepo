#if !defined(DOOR_MANIPULATOR_NORELEM_STEPPER_HPP)
#define DOOR_MANIPULATOR_NORELEM_STEPPER_HPP

#include <Arduino.h>

#include "pinout_defines.h"

namespace norelem_stepper
{
    // the time in ms the lock mechanism needs to open resp. close the lock
    #define LOCK_MECHANISM_TIME 700 // according to the datasheet a minimum of 600ms is required
    #define LOCK_AUTO_CLOSE_TIME_WHEN_DRAWER_NOT_OPENED 10000 //milliseconds

    class NorelemStepper
    {
        public:
            NorelemStepper() {}

            void initialize(uint8_t motor_start_pin, uint8_t motor_e1_pin, uint8_t motor_e2_pin, uint8_t motor_e3_pin)
            {
                this->motor_start_pin_ = motor_start_pin;
                this->motor_e1_pin_ = motor_e1_pin;
                this->motor_e2_pin_ = motor_e2_pin;
                this->motor_e3_pin_ = motor_e3_pin;

                pinMode(this->motor_start_pin_, OUTPUT);
                pinMode(this->motor_e1_pin_, OUTPUT);
                pinMode(this->motor_e2_pin_, OUTPUT);
                pinMode(this->motor_e3_pin_, OUTPUT);

                digitalWrite(this->motor_start_pin_, LOW);
                digitalWrite(this->motor_e1_pin_, HIGH);
                digitalWrite(this->motor_e2_pin_, HIGH);
                digitalWrite(this->motor_e3_pin_, HIGH);
                Serial.println("Norelem Stepper initialization completed!");
            }

            void switch_motor_state()
            {
                // in order to switch the motor state, we need to toggle the start pin from HIGH -> LOW -> HIGH
                this->set_motor_start_pin(false);
                delay(100); //TODO: Check how much delay is neccesarry
                this->set_motor_start_pin(true);
            }            

            void set_motor_input_pin(uint8_t input_id, bool state)
            {
                switch (input_id)
                {
                    case 1:
                        if (state)
                        {
                            digitalWrite(this->motor_e1_pin_, LOW);
                            Serial.println("Setting motor e1 pin low");
                        }
                        else
                        {
                            digitalWrite(this->motor_e1_pin_, HIGH);
                            Serial.println("Setting motor e1 pin high");
                        }
                        break;
                    
                    case 2:
                        if (state)
                        {
                            digitalWrite(this->motor_e2_pin_, LOW);
                            Serial.println("Setting motor e2 pin low");
                        }
                        else
                        {
                            digitalWrite(this->motor_e2_pin_, HIGH);
                            Serial.println("Setting motor e2 pin high");
                        }
                        break;

                    case 3:
                        if (state)
                        {
                            digitalWrite(this->motor_e3_pin_, LOW);
                            Serial.println("Setting motor e3 pin low");
                        }
                        else
                        {
                            digitalWrite(this->motor_e3_pin_, HIGH);
                            Serial.println("Setting motor e3 pin high");
                        }
                        break;
                    
                    default:
                        Serial.println("Requested motor input pin is not implemented!");
                        break;
                }
            }

        private:
            uint8_t motor_start_pin_;
            uint8_t motor_e1_pin_;
            uint8_t motor_e2_pin_;
            uint8_t motor_e3_pin_;

            void set_motor_start_pin(bool state)
            {
                if (state)
                {
                    digitalWrite(this->motor_start_pin_, HIGH);
                    Serial.println("Setting motor start pin high");
                }
                else
                {
                    digitalWrite(this->motor_start_pin_, LOW);
                    Serial.println("Setting motor start pin low");
                }
            }

    };
} // namespace norelem_stepper

#endif // DOOR_MANIPULATOR_NORELEM_STEPPER_HPP