#if !defined(NORELEM_STEPPER_HPP)
#define NORELEM_STEPPER_HPP

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

            void initialize(uint8_t motor_start_pin, uint8_t motor_input_1, uint8_t motor_input_2, uint8_t motor_input_3)
            {
                this->motor_start_pin_ = motor_start_pin;
                this->motor_input_1_ = motor_input_1;
                this->motor_input_2_ = motor_input_2;
                this->motor_input_3_ = motor_input_3;

                pinMode(this->motor_start_pin_, OUTPUT);
                pinMode(this->motor_input_1_, OUTPUT);
                pinMode(this->motor_input_2_, OUTPUT);
                pinMode(this->motor_input_3_, INPUT);

                digitalWrite(this->motor_start_pin_, LOW);
                digitalWrite(this->motor_input_1_, HIGH);
                digitalWrite(this->motor_input_2_, HIGH);
                digitalWrite(this->motor_input_3_, HIGH);
            }

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

        private:
            uint8_t motor_start_pin_;
            uint8_t motor_input_1_;
            uint8_t motor_input_2_;
            uint8_t motor_input_3_;

    };
} // namespace norelem_stepper

#endif // NORELEM_STEPPER_HPP