#if !defined(CAN_HPP)
#define CAN_HPP

#include <Arduino.h>
#include <mcp_can.h>

#include "pinout_defines.h"
#include "lock.hpp"

namespace can
{
    class Can
    {
        public:
            Can(uint32_t drawer_controller_id, lock::Lock *lock_1, lock::Lock *lock_2)
            {
                this->drawer_controller_id_ = drawer_controller_id;
                this->lock_1_ = lock_1;
                this->lock_2_ = lock_2;
            }

            void initialize_can_controller(void)
            {
                this->initialize_voltage_translator();

                if(this->CAN0_.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK)
                {
                    Serial.println("MCP2515 Initialized Successfully!");
                }
                else 
                {
                    Serial.println("Error Initializing MCP2515...");
                } 

                this->CAN0_.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted

                pinMode(MCP2515_INT, INPUT);  // Configuring pin for /INT input
            }

            void handle_receiving_can_msg()
            {
                if(!digitalRead(MCP2515_INT)) // If CAN0__INT pin is low, read receive buffer
                {  
                    Serial.println("Received CAN message!");

                    this->CAN0_.readMsgBuf(&this->rx_msg_id_, &this->rx_msg_dlc_, this->rx_data_buf_);

                    std::optional<robast_can_msgs::CanMessage> can_message = robast_can_msgs::decode_can_message(this->rx_msg_id_, this->rx_data_buf_, this->rx_msg_dlc_, this->can_db_.can_messages); 

                    if (can_message.has_value())
                    {
                        this->handle_can_msg(can_message.value());      
                    }
                    else
                    {
                        Serial.println("There is no CAN Message available in the CAN Database that corresponds to the msg id: ");
                        Serial.print(this->rx_msg_id_, HEX);
                    }
                }
            }

            void handle_sending_drawer_status_feedback(void)
            {
                bool is_endstop_switch_1_pushed = this->lock_1_->get_moving_average_drawer_closed_pin() > 0.9;
                bool is_lock_switch_1_pushed = this->lock_1_->get_moving_average_sensor_lock_pin() > 0.9;
                bool is_endstop_switch_2_pushed = this->lock_2_->get_moving_average_drawer_closed_pin() > 0.9;
                bool is_lock_switch_2_pushed = this->lock_2_->get_moving_average_sensor_lock_pin() > 0.9;

                this->handle_drawer_is_open_feedback(is_endstop_switch_1_pushed, is_endstop_switch_2_pushed);

                this->handle_drawer_is_closed_feedback(is_endstop_switch_1_pushed, is_lock_switch_1_pushed, is_endstop_switch_2_pushed, is_lock_switch_2_pushed);                
            }

        private:
            MCP_CAN CAN0_ = MCP_CAN(SPI_CS);

            uint32_t drawer_controller_id_;

            lock::Lock *lock_1_;
            lock::Lock *lock_2_;

            bool drawer_1_open_feedback_can_msg_sent_ = false;
            bool drawer_2_open_feedback_can_msg_sent_ = false;

            long unsigned int rx_msg_id_;
            uint8_t rx_msg_dlc_ = 0;
            uint8_t rx_data_buf_[8];

            unsigned long previous_millis_drawer_status_fb_ = 0;
            const uint16_t DEFAULT_INTERVAL_DRAWER_FEEDBACK_IN_MS_ = 1000;
            unsigned long interval_drawer_feedback_in_ms_ = DEFAULT_INTERVAL_DRAWER_FEEDBACK_IN_MS_;
            bool broadcast_feedback_ = false;

            robast_can_msgs::CanDb can_db_ = robast_can_msgs::CanDb();


            void initialize_voltage_translator(void)
            {
                pinMode(OE_TXB0104, OUTPUT);
                digitalWrite(OE_TXB0104, HIGH); // enable voltage level translator
            }

            void handle_lock_status(robast_can_msgs::CanMessage can_message)
            {
                if (can_message.get_can_signals().at(CAN_SIGNAL_OPEN_LOCK_1).get_data() == CAN_DATA_OPEN_LOCK)
                {
                    if (this->lock_1_->is_drawer_opening_in_progress())
                    {
                        Serial.println("Drawer 1 opening is already in progress, so lock won't be opened again!");
                    }
                    else
                    {
                        this->lock_1_->set_open_lock_current_step(true);
                        this->lock_1_->set_timestamp_last_lock_change();
                        this->lock_1_->set_drawer_opening_is_in_progress(true);
                    }
                }
                if (can_message.get_can_signals().at(CAN_SIGNAL_OPEN_LOCK_1).get_data() == CAN_DATA_CLOSE_LOCK)
                {
                    this->lock_1_->set_open_lock_current_step(false);
                }

                if (can_message.get_can_signals().at(CAN_SIGNAL_OPEN_LOCK_2).get_data() == CAN_DATA_OPEN_LOCK)
                {
                    if (this->lock_2_->is_drawer_opening_in_progress())
                    {
                        Serial.println("Drawer 2 opening is already in progress, so lock won't be opened again!");
                    }
                    else
                    {
                        this->lock_2_->set_open_lock_current_step(true);
                        this->lock_2_->set_timestamp_last_lock_change();
                        this->lock_2_->set_drawer_opening_is_in_progress(true);
                    }
                }
                if (can_message.get_can_signals().at(CAN_SIGNAL_OPEN_LOCK_2).get_data() == CAN_DATA_CLOSE_LOCK)
                {
                    this->lock_2_->set_open_lock_current_step(false);
                }
            }

            robast_can_msgs::CanMessage create_drawer_feedback_can_msg()
            {
                robast_can_msgs::CanMessage can_msg_drawer_feedback = this->can_db_.can_messages.at(CAN_MSG_DRAWER_FEEDBACK);
                std::vector can_signals_drawer_feedback = can_msg_drawer_feedback.get_can_signals();

                can_signals_drawer_feedback.at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).set_data(this->drawer_controller_id_);

                if (this->lock_1_->get_moving_average_drawer_closed_pin() > 0.9){
                    can_signals_drawer_feedback.at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_1_PUSHED).set_data(1);
                } else {
                    can_signals_drawer_feedback.at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_1_PUSHED).set_data(0);
                }

                if (this->lock_1_->get_moving_average_sensor_lock_pin() > 0.9){
                    can_signals_drawer_feedback.at(CAN_SIGNAL_IS_LOCK_SWITCH_1_PUSHED).set_data(1);
                } else {
                    can_signals_drawer_feedback.at(CAN_SIGNAL_IS_LOCK_SWITCH_1_PUSHED).set_data(0);
                }

                if (this->lock_2_->get_moving_average_drawer_closed_pin() > 0.9){
                    can_signals_drawer_feedback.at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_2_PUSHED).set_data(1);
                } else {
                    can_signals_drawer_feedback.at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_2_PUSHED).set_data(0);
                }

                if (this->lock_2_->get_moving_average_sensor_lock_pin() > 0.9){
                    can_signals_drawer_feedback.at(CAN_SIGNAL_IS_LOCK_SWITCH_2_PUSHED).set_data(1);
                } else {
                    can_signals_drawer_feedback.at(CAN_SIGNAL_IS_LOCK_SWITCH_2_PUSHED).set_data(0);
                }

                can_msg_drawer_feedback.set_can_signals(can_signals_drawer_feedback);

                return can_msg_drawer_feedback;
            }

            void sending_drawer_status_feedback(void)
            {
                robast_can_msgs::CanMessage can_msg_drawer_feedback = create_drawer_feedback_can_msg();

                try
                {
                    robast_can_msgs::CanFrame can_frame = robast_can_msgs::encode_can_message_into_can_frame(can_msg_drawer_feedback, can_db_.can_messages);
                    byte sndStat = this->CAN0_.sendMsgBuf(can_frame.get_id(), 0, can_frame.get_dlc(), can_frame.get_data());
                    if(sndStat == CAN_OK)
                    {
                    Serial.println("Message Sent Successfully!");
                    }
                    else
                    {
                    Serial.print("Error Sending Message... CAN Status is: ");
                    Serial.println(sndStat);
                    }  
                }
                catch (const std::invalid_argument& exception)
                {
                    Serial.print("Exception accurred while encoding CAN message into can frame. Exception message: ");
                    Serial.println(exception.what());
                }
            }

            void handle_can_msg(robast_can_msgs::CanMessage can_message)
            {
                if (can_message.get_id() == CAN_ID_DRAWER_LOCK)
                {
                    if (can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).get_data() == this->drawer_controller_id_)
                    {
                        this->handle_lock_status(can_message);
                    }

                    this->debug_prints_drawer_lock(can_message);
                }

                if (can_message.get_id() == CAN_ID_DRAWER_LED)
                {
                    if (can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).get_data() == this->drawer_controller_id_)
                    {
                        led_strip::select_led_strip_mode(can_message);
                    }

                    this->debug_prints_drawer_led(can_message);
                }
            }

            void handle_drawer_is_open_feedback(bool is_endstop_switch_1_pushed, bool is_endstop_switch_2_pushed)
            {
                bool is_drawer_1_open = !is_endstop_switch_1_pushed;
                bool is_drawer_2_open = !is_endstop_switch_2_pushed;
                if (this->lock_1_->is_drawer_opening_in_progress() && is_drawer_1_open && !this->drawer_1_open_feedback_can_msg_sent_)
                {
                    this->sending_drawer_status_feedback();
                    this->lock_1_->set_open_lock_current_step(false); // this makes sure the lock automatically closes as soon as the drawer is opened
                    this->drawer_1_open_feedback_can_msg_sent_ = true; // makes sure the feedback msg is only sent once
                }
                if (this->lock_2_->is_drawer_opening_in_progress() && is_drawer_2_open && !this->drawer_2_open_feedback_can_msg_sent_)
                {
                    this->sending_drawer_status_feedback();
                    this->lock_2_->set_open_lock_current_step(false); // this makes sure the lock automatically closes as soon as the drawer is opened
                    this->drawer_2_open_feedback_can_msg_sent_ = true; // makes sure the feedback msg is only sent once
                }
            }

            void handle_drawer_is_closed_feedback(bool is_endstop_switch_1_pushed, bool is_lock_switch_1_pushed, bool is_endstop_switch_2_pushed, bool is_lock_switch_2_pushed)
            {
                bool is_drawer_1_closed = is_endstop_switch_1_pushed && !is_lock_switch_1_pushed;
                bool is_drawer_2_closed = is_endstop_switch_2_pushed && !is_lock_switch_2_pushed;
                if (this->lock_1_->is_drawer_opening_in_progress() && is_drawer_1_closed && this->drawer_1_open_feedback_can_msg_sent_)
                {
                    this->sending_drawer_status_feedback();
                    this->lock_1_->set_drawer_opening_is_in_progress(false);
                    this->drawer_1_open_feedback_can_msg_sent_ = false; // reset this flag for the next opening of the drawer
                }
                if (this->lock_2_->is_drawer_opening_in_progress() && is_drawer_2_closed && this->drawer_2_open_feedback_can_msg_sent_)
                {
                    this->sending_drawer_status_feedback();
                    this->lock_2_->set_drawer_opening_is_in_progress(false);
                    this->drawer_2_open_feedback_can_msg_sent_ = false; // reset this flag for the next opening of the drawer
                }
            }

            void debug_prints_drawer_lock(robast_can_msgs::CanMessage can_message)
            {
                Serial.print("Standard ID: ");
                Serial.print(this->rx_msg_id_, HEX);
                Serial.print(" rx_dlc: ");
                Serial.print(uint8_t(this->rx_msg_dlc_), DEC);
                Serial.print(" DRAWER ID: ");
                Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).get_data(), HEX);
                Serial.print(" CAN_SIGNAL_OPEN_LOCK_1: ");
                Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_OPEN_LOCK_1).get_data(), BIN);
                Serial.print(" CAN_SIGNAL_OPEN_LOCK_2: ");
                Serial.println(can_message.get_can_signals().at(CAN_SIGNAL_OPEN_LOCK_2).get_data(), BIN);
            }

            void debug_prints_drawer_led(robast_can_msgs::CanMessage can_message)
            {
                Serial.print("Standard ID: ");
                Serial.print(this->rx_msg_id_, HEX);
                Serial.print(" rx_dlc: ");
                Serial.print(uint8_t(this->rx_msg_dlc_), DEC);
                Serial.print(" DRAWER ID: ");
                Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).get_data(), HEX);
                Serial.print(" LED RED: ");
                Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_LED_RED).get_data(), DEC);
                Serial.print(" LED GREEN: ");
                Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_LED_GREEN).get_data(), DEC);
                Serial.print(" LED BLUE: ");
                Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_LED_BLUE).get_data(), DEC);
                Serial.print(" LED BRIGHTNESS: ");
                Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_LED_BRIGHTNESS).get_data(), DEC);
                Serial.print(" LED MODE: ");
                Serial.println(can_message.get_can_signals().at(CAN_SIGNAL_LED_MODE).get_data(), DEC);
            }
    };
} // namespace can



#endif // CAN_HPP
