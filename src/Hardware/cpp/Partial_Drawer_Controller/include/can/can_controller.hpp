#ifndef DRAWER_CONTROLLER_CAN_CONTROLLER_HPP
#define DRAWER_CONTROLLER_CAN_CONTROLLER_HPP

#include <Arduino.h>
#include <CAN_config.h>
#include <ESP32CAN.h>

#include <memory>
#include <optional>

#include "can/can_db.hpp"
#include "can/can_helper.h"
#include "debug/debug.hpp"
#include "interfaces/i_gpio_wrapper.hpp"
#include "peripherals/pinout_defines.hpp"

namespace drawer_controller
{
  class CanController
  {
   public:
    CanController(uint32_t module_id,
                  std::shared_ptr<robast_can_msgs::CanDb> can_db,
                  std::shared_ptr<IGpioWrapper> gpio_wrapper);

    void initialize_can_controller(void);

    std::optional<robast_can_msgs::CanMessage> handle_receiving_can_msg();

    void send_can_message(robast_can_msgs::CanMessage can_msg);

   private:
    uint32_t _module_id;
    std::shared_ptr<robast_can_msgs::CanDb> _can_db;
    std::shared_ptr<IGpioWrapper> _gpio_wrapper;

    static const uint8_t _RX_QUEUE_SIZE = 10;   // Receive Queue size
  };
}   // namespace drawer_controller
#endif   // DRAWER_CONTROLLER_CAN_CONTROLLER_HPP
