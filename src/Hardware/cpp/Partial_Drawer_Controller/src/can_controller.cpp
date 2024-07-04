#include "can/can_controller.hpp"

// had a lot of linking errors when this was in the class, put it here and it works, no idea why
CAN_device_t CAN_cfg;

namespace drawer_controller
{

  CanController::CanController(uint32_t module_id,
                               std::shared_ptr<robast_can_msgs::CanDb> can_db,
                               std::shared_ptr<IGpioWrapper> gpio_wrapper)
      : _module_id{module_id}, _can_db{can_db}, _gpio_wrapper{gpio_wrapper} {};

  void CanController::initialize_can_controller(void)
  {
    // Important note: No idea why - when setting CAN rate to 250kbps at the jetson, we need to set it to 500kbps here
    CAN_cfg.speed = CAN_SPEED_500KBPS;
    CAN_cfg.tx_pin_id = TWAI_TX_PIN;
    CAN_cfg.rx_pin_id = TWAI_RX_PIN;
    CAN_cfg.rx_queue = xQueueCreate(_RX_QUEUE_SIZE, sizeof(CAN_frame_t));

    ESP32Can.CANInit();
  }

  std::optional<robast_can_msgs::CanMessage> CanController::handle_receiving_can_msg()
  {
    std::optional<robast_can_msgs::CanMessage> can_message;

    CAN_frame_t rx_frame;

    // Receive next CAN frame from queue
    if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE)
    {
      debug_println("Received CAN message!");

      if (rx_frame.FIR.B.FF == CAN_frame_std)
      {
        debug_println("New standard frame");
      }
      else
      {
        debug_println("New extended frame");
      }

      if (rx_frame.FIR.B.RTR == CAN_RTR)
      {
        debug_printf(" RTR from 0x%08X, DLC %d\r\n", rx_frame.MsgID, rx_frame.FIR.B.DLC);
      }
      else
      {
        debug_printf(" from 0x%08X, DLC %d, Data ", rx_frame.MsgID, rx_frame.FIR.B.DLC);
        for (int i = 0; i < rx_frame.FIR.B.DLC; i++)
        {
          debug_printf("0x%02X ", rx_frame.data.u8[i]);
        }
        debug_println("\n");

        can_message = robast_can_msgs::decode_can_message(
          rx_frame.MsgID, rx_frame.data.u8, rx_frame.FIR.B.DLC, this->_can_db->can_messages);

        if (can_message.has_value() &&
            can_message.value().get_can_signals().at(CAN_SIGNAL_MODULE_ID).get_data() == _module_id)
        {
          return can_message;
        }

        if (!can_message.has_value())
        {
          debug_println("There is no CAN Message available in the CAN Database that corresponds to the msg id: ");
          debug_print_with_base(rx_frame.MsgID, HEX);
          return can_message;
        }
        else
        {
          // Received msg that wasn't supposed for this module.
          can_message.reset();
          return can_message;
        }
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

      CAN_frame_t tx_frame;
      tx_frame.FIR.B.FF = CAN_frame_std;
      tx_frame.MsgID = can_frame.get_id();
      tx_frame.FIR.B.DLC = can_frame.get_dlc();

      for (uint8_t i = 0; i < can_frame.get_dlc(); i++)
      {
        tx_frame.data.u8[i] = can_frame.get_data()[i];
      }

      int result = ESP32Can.CANWriteFrame(&tx_frame);

      if (result == 0)
      {
        debug_println("Message Sent Successfully!");
      }
      else
      {
        debug_println("Error accured while sending CAN message!");
      }
    }
    catch (const std::invalid_argument &exception)
    {
      debug_print("Exception accurred while encoding CAN message into can frame. Exception message: ");
      debug_println(exception.what());
    }
  }

}   // namespace drawer_controller
