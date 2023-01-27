#if !defined(DOOR_MANIPULATOR_LOCK_HPP)
#define DOOR_MANIPULATOR_LOCK_HPP

#include <Arduino.h>

#include "pinout_defines.h"

namespace lock
{
    // the time in ms the lock mechanism needs to open resp. close the lock
    #define LOCK_MECHANISM_TIME 700 // according to the datasheet a minimum of 600ms is required
    #define LOCK_AUTO_CLOSE_TIME_WHEN_DRAWER_NOT_OPENED 10000 //milliseconds

    class Lock
    {
        public:
            Lock() {}

            void initialize_locks(uint8_t power_open_pin, uint8_t power_close_pin, uint8_t sensor_lock_pin, uint8_t sensor_drawer_closed_pin)
            {
                this->power_open_pin_ = power_open_pin;
                this->power_close_pin_ = power_close_pin;
                this->sensor_lock_pin_ = sensor_lock_pin;
                this->sensor_drawer_closed_pin_ = sensor_drawer_closed_pin;

                pinMode(this->power_open_pin_, OUTPUT);
                pinMode(this->power_close_pin_, OUTPUT);
                pinMode(this->sensor_lock_pin_, INPUT);
                pinMode(this->sensor_drawer_closed_pin_, INPUT);

                digitalWrite(this->power_open_pin_, LOW);
                digitalWrite(this->power_close_pin_, LOW);
            }

            void handle_lock_control()
            {
                // Mind that the state for open_lock_current_step_ is changed in the handle_lock_status function when a CAN msg is received
                bool change_lock_state = this->open_lock_current_step_ == this->open_lock_previous_step_ ? false : true;

                unsigned long current_timestamp = millis();
                unsigned long time_since_lock_state_was_changed = current_timestamp - this->timestamp_last_lock_change_;
                unsigned long time_since_lock_was_opened = current_timestamp - this->timestamp_last_lock_opening_;

                if (change_lock_state && (time_since_lock_state_was_changed >= LOCK_MECHANISM_TIME))
                {
                    this->open_lock_previous_step_ = this->open_lock_current_step_;
                    this->timestamp_last_lock_change_ = current_timestamp;
                    this->open_lock_current_step_ ? this->open_lock() : this->close_lock();                    
                }
                else if (!change_lock_state && (time_since_lock_state_was_changed >= LOCK_MECHANISM_TIME))
                {
                    // this makes sure, there is only a 5V pulse with the duration of LOCK_MECHANISM_TIME on the respective input of the lock
                    this->set_lock_output_low();
                }

                if (this->open_lock_current_step_ && (time_since_lock_was_opened > LOCK_AUTO_CLOSE_TIME_WHEN_DRAWER_NOT_OPENED))
                {
                    // Close the lock automatically after some seconds when drawer wasn't opened for safety reasons
                    this->set_open_lock_current_step(false);
                    this->set_drawer_opening_is_in_progress(false);
                    Serial.print(" time_since_lock_was_opened: ");
                    Serial.println(time_since_lock_was_opened, DEC);
                }
            }

            void set_open_lock_current_step(bool open_lock_current_step)
            {
                this->open_lock_current_step_ = open_lock_current_step;
            }

            void set_timestamp_last_lock_change()
            {
                this->timestamp_last_lock_opening_ = millis();
            }

            void set_drawer_opening_is_in_progress(bool drawer_opening_is_in_progress)
            {
                this->drawer_opening_is_in_progress_ = drawer_opening_is_in_progress;
            }

            bool is_drawer_opening_in_progress()
            {
                return this->drawer_opening_is_in_progress_;
            }

            void handle_reading_sensors()
            {
                // Tracking the moving average for the sensor pins helps to debounce them a little bit
                this->moving_average_sensor_lock_pin_ = 0.2 * digitalRead(this->sensor_lock_pin_) + 0.8 * this->moving_average_sensor_lock_pin_;
                this->moving_average_drawer_closed_pin_ = 0.2 * digitalRead(this->sensor_drawer_closed_pin_) + 0.8 * this->moving_average_drawer_closed_pin_;
            }

            float get_moving_average_sensor_lock_pin()
            {
                return this->moving_average_sensor_lock_pin_;
            }

            float get_moving_average_drawer_closed_pin()
            {
                return this->moving_average_drawer_closed_pin_;
            }


        private:
            uint8_t power_open_pin_;
            uint8_t power_close_pin_;
            uint8_t sensor_lock_pin_;
            uint8_t sensor_drawer_closed_pin_;

            bool open_lock_current_step_ = false;    // flag to store which state the locks should have
            bool open_lock_previous_step_ = false;   // flag to store state of the lock of the previous step

            bool drawer_opening_is_in_progress_ = false;

            unsigned long timestamp_last_lock_change_ = 0;
            unsigned long timestamp_last_lock_opening_ = 0;

            float moving_average_sensor_lock_pin_ = 0;
            float moving_average_drawer_closed_pin_ = 0;

            void open_lock()
            {
                digitalWrite(this->power_close_pin_, LOW);
                digitalWrite(this->power_open_pin_, HIGH);
            }

            void close_lock()
            {
                digitalWrite(this->power_open_pin_, LOW);
                digitalWrite(this->power_close_pin_, HIGH);
            }

            void set_lock_output_low()
            { 
                digitalWrite(this->power_open_pin_, LOW);
                digitalWrite(this->power_close_pin_, LOW);
            }
    };
} // namespace lock

#endif // DOOR_MANIPULATOR_LOCK_HPP