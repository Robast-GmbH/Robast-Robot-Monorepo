#ifndef CAN_TOOLBOX_CAN_CONTROLLER_MCP2515_HPP
#define CAN_TOOLBOX_CAN_CONTROLLER_MCP2515_HPP

#include <ACAN2515.h>

#include <memory>
#include <optional>

#include "can/can_db.hpp"
#include "can/can_helper.hpp"
#include "debug/debug.hpp"
#include "interfaces/i_gpio_wrapper.hpp"

namespace can_toolbox
{
  template <gpio_num_t spi_miso, gpio_num_t spi_mosi, gpio_num_t spi_clk, gpio_num_t spi_cs, gpio_num_t mcp2515_int_pin>
  class CanController
  {
   public:
    CanController(const uint32_t module_id,
                  const std::shared_ptr<robast_can_msgs::CanDb> can_db,
                  const std::shared_ptr<interfaces::IGpioWrapper> gpio_wrapper,
                  const uint8_t oe_txb0104_pin_id);

    void initialize_can_controller(void);

    std::optional<robast_can_msgs::CanMessage> handle_receiving_can_msg();

    void send_can_message(robast_can_msgs::CanMessage can_msg);

    bool is_message_available();

   private:
    const uint32_t _module_id;
    const std::shared_ptr<robast_can_msgs::CanDb> _can_db;
    const std::shared_ptr<interfaces::IGpioWrapper> _gpio_wrapper;
    const uint8_t _oe_txb0104_pin_id;

    static constexpr uint32_t _CAN_BIT_RATE = 250 * 1000;

    static constexpr uint32_t _QUARTZ_FREQUENCY = 8 * 1000 * 1000;   // 8 MHz

    static constexpr uint16_t _RECEIVE_BUFFER_SIZE = 150;

    uint64_t _rx_msg_id;
    uint8_t _rx_msg_dlc = 0;
    uint8_t _rx_data_buf[8];

    static std::unique_ptr<ACAN2515> _acan_2515;

    void initialize_voltage_translator(void);
  };

  /*********************************************************************************************************
 Implementations
 In C++ you need to include the implementation of the template class in the header file because the
 compiler needs to know the implementation of the template class when it is used in another file
*********************************************************************************************************/

  template <gpio_num_t spi_miso, gpio_num_t spi_mosi, gpio_num_t spi_clk, gpio_num_t spi_cs, gpio_num_t mcp2515_int_pin>
  std::unique_ptr<ACAN2515> CanController<spi_miso, spi_mosi, spi_clk, spi_cs, mcp2515_int_pin>::_acan_2515 =
    std::make_unique<ACAN2515>(spi_cs, SPI, mcp2515_int_pin);

  template <gpio_num_t spi_miso, gpio_num_t spi_mosi, gpio_num_t spi_clk, gpio_num_t spi_cs, gpio_num_t mcp2515_int_pin>
  CanController<spi_miso, spi_mosi, spi_clk, spi_cs, mcp2515_int_pin>::CanController(
    const uint32_t module_id,
    const std::shared_ptr<robast_can_msgs::CanDb> can_db,
    const std::shared_ptr<interfaces::IGpioWrapper> gpio_wrapper,
    const uint8_t oe_txb0104_pin_id)
      : _module_id{module_id}, _can_db{can_db}, _gpio_wrapper{gpio_wrapper}, _oe_txb0104_pin_id{oe_txb0104_pin_id}
  {
    initialize_can_controller();
  };

  template <gpio_num_t spi_miso, gpio_num_t spi_mosi, gpio_num_t spi_clk, gpio_num_t spi_cs, gpio_num_t mcp2515_int_pin>
  void CanController<spi_miso, spi_mosi, spi_clk, spi_cs, mcp2515_int_pin>::initialize_can_controller(void)
  {
    initialize_voltage_translator();

    SPI.begin(spi_clk, spi_miso, spi_mosi, spi_cs);

    ACAN2515Settings settings2515(_QUARTZ_FREQUENCY, _CAN_BIT_RATE);

    settings2515.mReceiveBufferSize = _RECEIVE_BUFFER_SIZE;

    const uint32_t errorCode2515 = _acan_2515->begin(settings2515,
                                                     []
                                                     {
                                                       _acan_2515->isr();
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

    pinMode(mcp2515_int_pin, INPUT);   // Configuring pin for /INT input
  }

  template <gpio_num_t spi_miso, gpio_num_t spi_mosi, gpio_num_t spi_clk, gpio_num_t spi_cs, gpio_num_t mcp2515_int_pin>
  void CanController<spi_miso, spi_mosi, spi_clk, spi_cs, mcp2515_int_pin>::initialize_voltage_translator(void)
  {
    _gpio_wrapper->set_pin_mode(_oe_txb0104_pin_id, interfaces::gpio::IS_OUTPUT);
    _gpio_wrapper->digital_write(_oe_txb0104_pin_id, HIGH);
  }

  template <gpio_num_t spi_miso, gpio_num_t spi_mosi, gpio_num_t spi_clk, gpio_num_t spi_cs, gpio_num_t mcp2515_int_pin>
  std::optional<robast_can_msgs::CanMessage>
  CanController<spi_miso, spi_mosi, spi_clk, spi_cs, mcp2515_int_pin>::handle_receiving_can_msg()
  {
    std::optional<robast_can_msgs::CanMessage> can_message;
    if (_acan_2515->available())
    {
      debug_println("Received CAN message!");

      CANMessage frame;
      _acan_2515->receive(frame);

      _rx_msg_id = frame.id;
      _rx_msg_dlc = frame.len;

      can_message = robast_can_msgs::decode_can_message(_rx_msg_id, frame.data, _rx_msg_dlc, _can_db->can_messages);

      if (can_message.has_value() && can_message.value()
                                         .get_can_signals()
                                         .at(robast_can_msgs::can_signal::id::drawer_unlock::MODULE_ID)
                                         .get_data() == _module_id)
      {
        return can_message;
      }

      if (!can_message.has_value())
      {
        debug_println("There is no CAN Message available in the CAN Database that corresponds to the msg id: ");
        debug_print_with_base(_rx_msg_id, HEX);
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

  template <gpio_num_t spi_miso, gpio_num_t spi_mosi, gpio_num_t spi_clk, gpio_num_t spi_cs, gpio_num_t mcp2515_int_pin>
  void CanController<spi_miso, spi_mosi, spi_clk, spi_cs, mcp2515_int_pin>::send_can_message(
    robast_can_msgs::CanMessage can_msg)
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

      const bool ok = _acan_2515->tryToSend(mcp2515_frame);
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

  template <gpio_num_t spi_miso, gpio_num_t spi_mosi, gpio_num_t spi_clk, gpio_num_t spi_cs, gpio_num_t mcp2515_int_pin>
  bool CanController<spi_miso, spi_mosi, spi_clk, spi_cs, mcp2515_int_pin>::is_message_available()
  {
    return _acan_2515->available();
  }

}   // namespace can_toolbox

#endif   // CAN_TOOLBOX_CAN_CONTROLLER_MCP2515_HPP