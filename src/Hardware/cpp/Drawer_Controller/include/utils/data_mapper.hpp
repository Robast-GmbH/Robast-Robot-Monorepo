#ifndef DRAWER_CONTROLLER_DATA_MAPPER_HPP
#define DRAWER_CONTROLLER_DATA_MAPPER_HPP

#include <Arduino.h>

#include "can/can_db.hpp"
#include "led/led_header.hpp"
#include "led/led_state.hpp"

namespace drawer_controller
{
  class DataMapper
  {
   public:
    DataMapper() = default;

    LedState create_led_state(const robast_can_msgs::CanMessage msg);

    LedHeader create_led_header(const robast_can_msgs::CanMessage msg);

    // TODO@Jacob: add more data mapping here and remove can_in functions from drawer.hpp and electrical_drawer.hpp

   private:
  };

}   // namespace drawer_controller

#endif   // DRAWER_CONTROLLER_DATA_MAPPER_HPP