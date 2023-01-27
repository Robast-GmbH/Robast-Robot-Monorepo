#if !defined(DOOR_MANIPULATOR_CAN_HPP)
#define DOOR_MANIPULATOR_CAN_HPP

#include <Arduino.h>
#include <mcp_can.h>

#include "pinout_defines.h"
#include "norelem_stepper.hpp"
#include "can/can_db.hpp"
#include "can/can_helper.h"

namespace can
{
    class Can
    {
        public:
            Can(uint32_t drawer_controller_id, norelem_stepper::NorelemStepper *stepper_1, norelem_stepper::NorelemStepper *stepper_2, norelem_stepper::NorelemStepper *stepper_3)
            {
                this->drawer_controller_id_ = drawer_controller_id;
                this->stepper_1_ = stepper_1;
                this->stepper_2_ = stepper_2;
                this->stepper_3_ = stepper_3;
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

        private:
            MCP_CAN CAN0_ = MCP_CAN(SPI_CS);

            uint32_t drawer_controller_id_;

            norelem_stepper::NorelemStepper *stepper_1_;
            norelem_stepper::NorelemStepper *stepper_2_;
            norelem_stepper::NorelemStepper *stepper_3_;

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

            void handle_stepper_status(robast_can_msgs::CanMessage can_message)
            {
                uint8_t motor_id = can_message.get_can_signals().at(CAN_SIGNAL_MOTOR_ID).get_data();
                bool target_state_motor_e1 = can_message.get_can_signals().at(CAN_SIGNAL_MOTOR_E1).get_data() == 1;
                bool target_state_motor_e2 = can_message.get_can_signals().at(CAN_SIGNAL_MOTOR_E2).get_data() == 1;
                bool target_state_motor_e3 = can_message.get_can_signals().at(CAN_SIGNAL_MOTOR_E3).get_data() == 1;
                bool target_state_motor_e4 = can_message.get_can_signals().at(CAN_SIGNAL_MOTOR_E4).get_data() == 1;

                switch (motor_id)
                {
                    case 1:
                        this->stepper_1_->set_motor_input_pin(1, target_state_motor_e1);
                        this->stepper_1_->set_motor_input_pin(2, target_state_motor_e2);
                        this->stepper_1_->set_motor_input_pin(3, target_state_motor_e3);
                        this->stepper_1_->set_motor_input_pin(4, target_state_motor_e4);
                        this->stepper_1_->switch_motor_state();
                        break;
                    
                    case 2:
                        this->stepper_2_->set_motor_input_pin(1, target_state_motor_e1);
                        this->stepper_2_->set_motor_input_pin(2, target_state_motor_e2);
                        this->stepper_2_->set_motor_input_pin(3, target_state_motor_e3);
                        this->stepper_2_->set_motor_input_pin(4, target_state_motor_e4);
                        this->stepper_2_->switch_motor_state();
                        break;

                    case 3:
                        this->stepper_3_->set_motor_input_pin(1, target_state_motor_e1);
                        this->stepper_3_->set_motor_input_pin(2, target_state_motor_e2);
                        this->stepper_3_->set_motor_input_pin(3, target_state_motor_e3);
                        this->stepper_3_->set_motor_input_pin(4, target_state_motor_e4);
                        this->stepper_3_->switch_motor_state();
                        break;
                    
                    default:
                        break;
                }
            }

            void handle_can_msg(robast_can_msgs::CanMessage can_message)
            {
                if (can_message.get_id() == CAN_ID_DOOR_MANIPULATOR)
                {
                    if (can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).get_data() == this->drawer_controller_id_)
                    {
                        this->handle_stepper_status(can_message);
                    }

                    this->debug_prints_requested_stepper_state(can_message);
                }
            }

            void debug_prints_requested_stepper_state(robast_can_msgs::CanMessage can_message)
            {
                Serial.print("Standard ID: ");
                Serial.print(this->rx_msg_id_, HEX);
                Serial.print(" rx_dlc: ");
                Serial.print(uint8_t(this->rx_msg_dlc_), DEC);
                Serial.print(" DRAWER ID: ");
                Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).get_data(), HEX);
                Serial.print(" CAN_SIGNAL_MOTOR_ID: ");
                Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_MOTOR_ID).get_data(), BIN);
                Serial.print(" CAN_SIGNAL_MOTOR_E1: ");
                Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_MOTOR_E1).get_data(), BIN);
                Serial.print(" CAN_SIGNAL_MOTOR_E2: ");
                Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_MOTOR_E2).get_data(), BIN);
                Serial.print(" CAN_SIGNAL_MOTOR_E3: ");
                Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_MOTOR_E3).get_data(), BIN);
                Serial.print(" CAN_SIGNAL_MOTOR_E4: ");
                Serial.println(can_message.get_can_signals().at(CAN_SIGNAL_MOTOR_E4).get_data(), BIN);
            }

    };
} // namespace can



#endif // DOOR_MANIPULATOR_CAN_HPP
