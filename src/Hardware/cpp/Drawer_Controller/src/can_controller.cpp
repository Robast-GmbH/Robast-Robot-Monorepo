#include "can/can_controller.hpp"

namespace drawer_controller
{
  ACAN2515 acan_2515(SPI_CS, SPI, MCP2515_INT);

  CanController::CanController(uint32_t module_id,
                               std::shared_ptr<robast_can_msgs::CanDb> can_db,
                               std::shared_ptr<IGpioWrapper> gpio_wrapper,
                               uint8_t oe_txb0104_pin_id,
                               bool gpio_output_state)
      : _module_id{module_id},
        _can_db{can_db},
        _gpio_wrapper{gpio_wrapper},
        _oe_txb0104_pin_id{oe_txb0104_pin_id},
        _gpio_output_state{gpio_output_state} {};

  void CanController::initialize_can_controller(void)
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
      debug_println("ACAN2515 configuration: ok");
    }
    else
    {
      debug_print("ACAN2515 configuration error 0x");
      debug_println_with_base(errorCode2515, HEX);
    }

    pinMode(MCP2515_INT, INPUT);   // Configuring pin for /INT input
  }

  std::optional<robast_can_msgs::CanMessage> CanController::handle_receiving_can_msg()
  {
    std::optional<robast_can_msgs::CanMessage> can_message;
    if (acan_2515.available())
    {
      debug_println("Received CAN message!");

      CANMessage frame;
      acan_2515.receive(frame);

      this->_rx_msg_id = frame.id;
      this->_rx_msg_dlc = frame.len;

      can_message = robast_can_msgs::decode_can_message(
        this->_rx_msg_id, frame.data, this->_rx_msg_dlc, this->_can_db->can_messages);

      if (can_message.has_value() &&
          can_message.value().get_can_signals().at(CAN_SIGNAL_MODULE_ID).get_data() == _module_id)
      {
        return can_message;
      }

      if (!can_message.has_value())
      {
        debug_println("There is no CAN Message available in the CAN Database that corresponds to the msg id: ");
        debug_print_with_base(this->_rx_msg_id, HEX);
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

  void CanController::send_can_message(robast_can_msgs::CanMessage can_msg)
  {
    try
    {
      robast_can_msgs::CanFrame can_frame =
        robast_can_msgs::encode_can_message_into_can_frame(can_msg, _can_db->can_messages);

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
        debug_println("Message Sent Successfully!");
      }
      else
      {
        debug_println("Error accured while sending CAn message!");
      }
    }
    catch (const std::invalid_argument &exception)
    {
      debug_print("Exception accurred while encoding CAN message into can frame. Exception message: ");
      debug_println(exception.what());
    }
  }

  bool CanController::is_message_available()
  {
    return acan_2515.available();
  }

  void CanController::initialize_voltage_translator(void)
  {
    _gpio_wrapper->set_pin_mode(_oe_txb0104_pin_id, _gpio_output_state);
    _gpio_wrapper->digital_write(_oe_txb0104_pin_id, HIGH);
  }
}   // namespace drawer_controller
