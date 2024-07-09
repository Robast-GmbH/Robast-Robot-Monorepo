#ifndef DRAWER_CONTROLLER_DATA_MAPPER_HPP
#define DRAWER_CONTROLLER_DATA_MAPPER_HPP

#include "can/can_db.hpp"
#include "led/led_header.hpp"
#include "led/led_state.hpp"
#include "utils/e_drawer_task.hpp"

namespace drawer_controller
{
  class DataMapper
  {
   public:
    DataMapper() = default;

    LedState create_led_state(const robast_can_msgs::CanMessage msg) const;

    LedHeader create_led_header(const robast_can_msgs::CanMessage msg) const;

    EDrawerTask create_e_drawer_task(const robast_can_msgs::CanMessage msg) const;
  };

}   // namespace drawer_controller

#endif   // DRAWER_CONTROLLER_DATA_MAPPER_HPP