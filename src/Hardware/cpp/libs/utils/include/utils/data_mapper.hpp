#ifndef UTILS_DATA_MAPPER_HPP
#define UTILS_DATA_MAPPER_HPP

#include "can/can_db.hpp"
#include "led/led_header.hpp"
#include "led/led_state.hpp"
#include "utils/e_drawer_task.hpp"

namespace utils
{
  class DataMapper
  {
   public:
    DataMapper() = default;

    led::LedState create_led_state(const robast_can_msgs::CanMessage msg) const;

    led::LedHeader create_led_header(const robast_can_msgs::CanMessage msg) const;

    EDrawerTask create_e_drawer_task(const robast_can_msgs::CanMessage msg) const;
  };

}   // namespace utils

#endif   // UTILS_DATA_MAPPER_HPP