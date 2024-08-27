#ifndef CAN_TOOLBOX_CAN_CONTROLLER_HPP
#define CAN_TOOLBOX_CAN_CONTROLLER_HPP

#include <Arduino.h>
#include <CAN_config.h>
#include <ESP32CAN.h>

#include <memory>
#include <optional>

#include "can/can_db.hpp"
#include "can/can_helper.hpp"
#include "debug/debug.hpp"

namespace can_toolbox
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

    void initialize_can_controller(void);

  private:
    const uint32_t _module_id;
    const std::shared_ptr<robast_can_msgs::CanDb> _can_db;
    const gpio_num_t _twai_tx_pin;
    const gpio_num_t _twai_rx_pin;

    static constexpr uint8_t _RX_QUEUE_SIZE = 255;               // I think 255 is the maximum size of the queue
    static constexpr uint8_t _RX_QUEUE_MINIMUM_EMPTY_SPACE = 55; // this is the value where we start to drop msgs

    void handle_emerging_queue_overflow(void);
  };
} // namespace can_toolbox
#endif // CAN_TOOLBOX_CAN_CONTROLLER_HPP
