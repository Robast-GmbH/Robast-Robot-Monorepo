#ifndef DRAWER_CONTROLLER_CAN_HPP
#define DRAWER_CONTROLLER_CAN_HPP

#include <ACAN2515.h>
#include <Arduino.h>

#include "can/can_db.hpp"
#include "can/can_helper.h"
#include "i_gpio_wrapper.hpp"
#include "pinout_defines.h"

namespace drawer_controller
{
  ACAN2515 acan_2515(SPI_CS, SPI, MCP2515_INT);

  class Can
  {
   public:
    Can(uint32_t module_id,
        std::shared_ptr<IGpioWrapper> gpio_wrapper,
        uint8_t oe_txb0104_pin_id,
        bool gpio_output_state)
        : _module_id{module_id},
          _gpio_wrapper{gpio_wrapper},
          _oe_txb0104_pin_id{oe_txb0104_pin_id},
          _gpio_output_state{gpio_output_state} {};

    void initialize_can_controller(void)
    {
      this->initialize_voltage_translator();

      //--- Configure SPI
      SPI.begin(SPI_CLK, SPI_MISO, SPI_MOSI, SPI_CS);

      ACAN2515Settings settings2515(this->_QUARTZ_FREQUENCY, this->_CAN_BIT_RATE);

      const uint32_t errorCode2515 = acan_2515.begin(settings2515,
                                                     []
                                                     {
                                                       acan_2515.isr();
                                                     });
      if (errorCode2515 == 0)
      {
        Serial.println("ACAN2515 configuration: ok");
      }
      else
      {
        Serial.print("ACAN2515 configuration error 0x");
        Serial.println(errorCode2515, HEX);
      }

      pinMode(MCP2515_INT, INPUT);   // Configuring pin for /INT input
    }

    std::optional<robast_can_msgs::CanMessage> handle_receiving_can_msg()
    {
      std::optional<robast_can_msgs::CanMessage> can_message;
      if (acan_2515.available())
      {
        Serial.println("Received CAN message!");

        CANMessage frame;
        acan_2515.receive(frame);

        this->_rx_msg_id = frame.id;
        this->_rx_msg_dlc = frame.len;

        can_message = robast_can_msgs::decode_can_message(
            this->_rx_msg_id, frame.data, this->_rx_msg_dlc, this->_can_db.can_messages);

        if (can_message.has_value() &&
            can_message.value().get_can_signals().at(CAN_SIGNAL_MODULE_ID).get_data() == _module_id)
        {
          return can_message;
        }

        if (!can_message.has_value())
        {
          Serial.println("There is no CAN Message available in the CAN Database that corresponds to the msg id: ");
          Serial.print(this->_rx_msg_id, HEX);
          return can_message;
        }
        else
        {
          // Received msg that wasn't supposed for this module.
          can_message.reset();
          return can_message;
        }
      }
      return can_message;
    }

    void send_can_message(robast_can_msgs::CanMessage can_msg)
    {
      try
      {
        robast_can_msgs::CanFrame can_frame =
            robast_can_msgs::encode_can_message_into_can_frame(can_msg, _can_db.can_messages);

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
      catch (const std::invalid_argument &exception)
      {
        Serial.print("Exception accurred while encoding CAN message into can frame. Exception message: ");
        Serial.println(exception.what());
      }
    }

    bool is_message_available()
    {
      return acan_2515.available();
    }

   private:
    uint32_t _module_id;
    std::shared_ptr<IGpioWrapper> _gpio_wrapper;
    uint8_t _oe_txb0104_pin_id;
    bool _gpio_output_state;

    static const uint32_t _CAN_BIT_RATE = 250 * 1000;

    static const uint32_t _QUARTZ_FREQUENCY = 8 * 1000 * 1000;   // 8 MHz

    long unsigned int _rx_msg_id;
    uint8_t _rx_msg_dlc = 0;
    uint8_t _rx_data_buf[8];

    robast_can_msgs::CanDb _can_db = robast_can_msgs::CanDb();

    void initialize_voltage_translator(void)
    {
      _gpio_wrapper->set_pin_mode(_oe_txb0104_pin_id, _gpio_output_state);
      _gpio_wrapper->digital_write(_oe_txb0104_pin_id, HIGH);
    }
  };
}   // namespace drawer_controller
#endif   // DRAWER_CONTROLLER_CAN_HPP
