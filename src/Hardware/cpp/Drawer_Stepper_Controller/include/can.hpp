#ifndef DRAWER_STEPPER_CONTROLLER_CAN_HPP
#define DRAWER_STEPPER_CONTROLLER_CAN_HPP

#include <Arduino.h>
#include <ACAN2515.h>

#include "pinout_defines.h"
#include "can/can_db.hpp"
#include "can/can_helper.h"

    ACAN2515 acan_2515 (SPI_CS, SPI, MCP2515_INT) ;

    class Can 
    {
        public:
              Can(uint32_t module_id)
            {
                this->module_id_ = module_id;
            }

            void initialize_can_controller(void)
            {
                this->initialize_voltage_translator();

                //--- Configure SPI
                SPI.begin(SPI_CLK, SPI_MISO, SPI_MOSI, SPI_CS);

                ACAN2515Settings settings2515 (this->QUARTZ_FREQUENCY_, this->CAN_BIT_RATE_);

                const uint32_t errorCode2515 = acan_2515.begin (settings2515, [] { acan_2515.isr () ; });
                if (errorCode2515 == 0)
                {
                    Serial.println ("ACAN2515 configuration: ok") ;
                }
                else
                {
                    Serial.print ("ACAN2515 configuration error 0x") ;
                    Serial.println (errorCode2515, HEX) ;
                }

                pinMode(MCP2515_INT, INPUT);  // Configuring pin for /INT input
            }

            std::optional<robast_can_msgs::CanMessage> handle_receiving_can_msg()
            {
                std::optional<robast_can_msgs::CanMessage> can_message;
                if (acan_2515.available ())
                {
                    Serial.println("Received CAN message!");

                    CANMessage frame;
                    acan_2515.receive(frame);

                    this->rx_msg_id_ = frame.id;
                    this->rx_msg_dlc_ = frame.len;

                    can_message = robast_can_msgs::decode_can_message(this->rx_msg_id_, frame.data, this->rx_msg_dlc_, this->can_db_.can_messages); 

                    if (can_message.has_value()&&can_message.value().get_can_signals().at(CAN_SIGNAL_MODULE_ID).get_data() == module_id_)
                    {
                        return can_message;     
                    }
                    
                    if(!can_message.has_value())
                    {
                        Serial.println("There is no CAN Message available in the CAN Database that corresponds to the msg id: ");
                        Serial.print(this->rx_msg_id_, HEX);
                        return can_message;
                    }else{
                        // Received msg that wasn't supposed for this module.
                        can_message.reset();
                        return can_message;
                    }
                }
                return can_message;
            }


            void send_can_message(robast_can_msgs::CanMessage can_msg){
                  try
                {
                    robast_can_msgs::CanFrame can_frame = robast_can_msgs::encode_can_message_into_can_frame(can_msg, can_db_.can_messages);

                    CANMessage mcp2515_frame;
                    mcp2515_frame.id = can_frame.get_id();
                    mcp2515_frame.len = can_frame.get_dlc();
                    for (uint8_t i = 0; i < can_frame.get_dlc(); i++)
                    {
                        mcp2515_frame.data[i] = can_frame.get_data()[i];
                    }

                    const bool ok = acan_2515.tryToSend(mcp2515_frame);
                    if (ok)
                    {
                        Serial.println("Message Sent Successfully!");
                    }
                    else
                    {
                        Serial.println("Error accured while sending CAn message!");
                    }
                }
                catch (const std::invalid_argument& exception)
                {
                    Serial.print("Exception accurred while encoding CAN message into can frame. Exception message: ");
                    Serial.println(exception.what());
                }
            }

            bool is_message_available(){
                return acan_2515.available();
            }

        private:
            static const uint32_t CAN_BIT_RATE_ = 250 * 1000 ;

            static const byte MCP2515_CS_  = SPI_CS ; // CS input of MCP2515
            static const byte MCP2515_SCK_ = SPI_CLK ; // SCK input of MCP2515
            static const byte MCP2515_SI_  = SPI_MOSI ; // SI input of MCP2515
            static const byte MCP2515_SO_  = SPI_MISO ; // SO output of MCP2515

            static const uint32_t QUARTZ_FREQUENCY_ = 8 * 1000 * 1000 ; // 8 MHz

            ACAN2515 CAN0_ = ACAN2515(MCP2515_CS_, SPI, MCP2515_INT);

            uint32_t module_id_;

            long unsigned int rx_msg_id_;
            uint8_t rx_msg_dlc_ = 0;
            uint8_t rx_data_buf_[8];

            robast_can_msgs::CanDb can_db_ = robast_can_msgs::CanDb();

            void initialize_voltage_translator(void)
            {
                pinMode(OE_TXB0104, OUTPUT);
                digitalWrite(OE_TXB0104, HIGH); // enable voltage level translator
            }
         
    };



#endif // DRAWER_CONTROLLER_CAN_HPP
