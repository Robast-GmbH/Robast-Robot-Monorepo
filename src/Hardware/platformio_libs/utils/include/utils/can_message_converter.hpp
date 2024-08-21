#ifndef UTILS_CAN_MESSAGE_CONVERTER_HPP
#define UTILS_CAN_MESSAGE_CONVERTER_HPP

#include "can/can_db.hpp"
#include "led/led_header.hpp"
#include "led/led_state.hpp"
#include "utils/e_drawer_task.hpp"

namespace utils
{
  class CanMessageConverter
  {
   public:
    CanMessageConverter() = default;

    led::LedState convert_to_led_state(const robast_can_msgs::CanMessage msg) const;

    led::LedHeader convert_to_led_header(const robast_can_msgs::CanMessage msg) const;

    EDrawerTask convert_to_e_drawer_task(const robast_can_msgs::CanMessage msg) const;
  };

}   // namespace utils

#endif   // UTILS_CAN_MESSAGE_CONVERTER_HPP