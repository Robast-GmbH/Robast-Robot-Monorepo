#include "utils/data_mapper.hpp"

namespace drawer_controller
{
  LedState DataMapper::create_led_state(const robast_can_msgs::CanMessage msg) const
  {
    return LedState(msg.get_can_signals().at(CAN_SIGNAL_SINGLE_LED_STATE_RED).get_data(),
                    msg.get_can_signals().at(CAN_SIGNAL_SINGLE_LED_STATE_GREEN).get_data(),
                    msg.get_can_signals().at(CAN_SIGNAL_SINGLE_LED_STATE_BLUE).get_data(),
                    msg.get_can_signals().at(CAN_SIGNAL_SINGLE_LED_STATE_BRIGHTNESS).get_data());
  }

  LedHeader DataMapper::create_led_header(const robast_can_msgs::CanMessage msg) const
  {
    return LedHeader(msg.get_can_signals().at(CAN_SIGNAL_NUM_OF_LEDS).get_data(),
                     msg.get_can_signals().at(CAN_SIGNAL_START_INDEX).get_data(),
                     msg.get_can_signals().at(CAN_SIGNAL_FADE_TIME_IN_HUNDREDS_OF_MS).get_data());
  }

  EDrawerTask DataMapper::create_e_drawer_task(const robast_can_msgs::CanMessage msg) const
  {
    return EDrawerTask(msg.get_can_signals().at(CAN_SIGNAL_DRAWER_TARGET_POSITION).get_data(),
                       msg.get_can_signals().at(CAN_SIGNAL_DRAWER_SPEED).get_data(),
                       msg.get_can_signals().at(CAN_SIGNAL_DRAWER_STALL_GUARD_ENABLE).get_data());
  }

}   // namespace drawer_controller