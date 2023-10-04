#ifndef DRAWER_CONTROLLER_CAN_CONTROLLER_HPP
#define DRAWER_CONTROLLER_CAN_CONTROLLER_HPP

#include <ACAN2515.h>
#include <Arduino.h>

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
                  std::shared_ptr<IGpioWrapper> gpio_wrapper,
                  uint8_t oe_txb0104_pin_id,
                  bool gpio_output_state);

    void initialize_can_controller(void);

    std::optional<robast_can_msgs::CanMessage> handle_receiving_can_msg();

    void send_can_message(robast_can_msgs::CanMessage can_msg);

    bool is_message_available();

   private:
    uint32_t _module_id;
    std::shared_ptr<robast_can_msgs::CanDb> _can_db;
    std::shared_ptr<IGpioWrapper> _gpio_wrapper;
    uint8_t _oe_txb0104_pin_id;
    bool _gpio_output_state;

    static const uint32_t _CAN_BIT_RATE = 250 * 1000;

    static const uint32_t _QUARTZ_FREQUENCY = 8 * 1000 * 1000;   // 8 MHz

    uint64_t _rx_msg_id;
    uint8_t _rx_msg_dlc = 0;
    uint8_t _rx_data_buf[8];

    void initialize_voltage_translator(void);
  };
}   // namespace drawer_controller
#endif   // DRAWER_CONTROLLER_CAN_CONTROLLER_HPP
