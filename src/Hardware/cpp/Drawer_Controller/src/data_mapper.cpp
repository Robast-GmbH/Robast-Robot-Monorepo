#include "utils/data_mapper.hpp"

namespace drawer_controller
{
  LedState DataMapper::create_led_state(const robast_can_msgs::CanMessage msg)
  {
    return LedState(msg.get_can_signals().at(CAN_SIGNAL_SINGLE_LED_STATE_RED).get_data(),
                    msg.get_can_signals().at(CAN_SIGNAL_SINGLE_LED_STATE_GREEN).get_data(),
                    msg.get_can_signals().at(CAN_SIGNAL_SINGLE_LED_STATE_BLUE).get_data(),
                    msg.get_can_signals().at(CAN_SIGNAL_SINGLE_LED_STATE_BRIGHTNESS).get_data());
  }

}   // namespace drawer_controller