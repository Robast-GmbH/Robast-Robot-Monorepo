#ifndef CAN_CONTROLLER_CAN_CONTROLLER_HPP
#define CAN_CONTROLLER_CAN_CONTROLLER_HPP

#include <Arduino.h>
#include <CAN_config.h>
#include <ESP32CAN.h>

#include <memory>
#include <optional>

#include "can/can_db.hpp"
#include "can/can_helper.hpp"
#include "debug/debug.hpp"

namespace can_controller
{
  class CanController
  {
   public:
    CanController(const uint32_t module_id,
                  const std::shared_ptr<robast_can_msgs::CanDb> can_db,
                  const gpio_num_t twai_tx_pin,
                  const gpio_num_t twai_rx_pin);

    std::optional<robast_can_msgs::CanMessage> handle_receiving_can_msg();

    void send_can_message(robast_can_msgs::CanMessage can_msg);

   private:
    const uint32_t _module_id;
    const std::shared_ptr<robast_can_msgs::CanDb> _can_db;
    const gpio_num_t _twai_tx_pin;
    const gpio_num_t _twai_rx_pin;

    static const uint8_t _RX_QUEUE_SIZE = 150;   // Receive Queue size

    void initialize_can_controller(void);
  };
}   // namespace can_controller
#endif   // CAN_CONTROLLER_CAN_CONTROLLER_HPP
