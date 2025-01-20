#include "utils/can_message_converter.hpp"

namespace utils
{
  led::LedState CanMessageConverter::convert_to_led_state(const robast_can_msgs::CanMessage msg) const
  {
    return led::LedState(
      msg.get_can_signals().at(robast_can_msgs::can_signal::id::led_state::LED_STATE_RED).get_data(),
      msg.get_can_signals().at(robast_can_msgs::can_signal::id::led_state::LED_STATE_GREEN).get_data(),
      msg.get_can_signals().at(robast_can_msgs::can_signal::id::led_state::LED_STATE_BLUE).get_data(),
      msg.get_can_signals().at(robast_can_msgs::can_signal::id::led_state::LED_STATE_BRIGHTNESS).get_data(),
      msg.get_can_signals().at(robast_can_msgs::can_signal::id::led_state::IS_GROUP_STATE).get_data(),
      msg.get_can_signals().at(robast_can_msgs::can_signal::id::led_state::ACK_REQUESTED).get_data());
  }

  led::LedHeader CanMessageConverter::convert_to_led_header(const robast_can_msgs::CanMessage msg) const
  {
    return led::LedHeader(
      msg.get_can_signals().at(robast_can_msgs::can_signal::id::led_header::NUM_OF_LEDS).get_data(),
      msg.get_can_signals().at(robast_can_msgs::can_signal::id::led_header::START_INDEX).get_data(),
      msg.get_can_signals().at(robast_can_msgs::can_signal::id::led_header::FADE_TIME_IN_HUNDREDS_OF_MS).get_data());
  }

  EDrawerTask CanMessageConverter::convert_to_e_drawer_task(const robast_can_msgs::CanMessage msg) const
  {
    return EDrawerTask(
      msg.get_can_signals().at(robast_can_msgs::can_signal::id::e_drawer_task::DRAWER_TARGET_POSITION).get_data(),
      msg.get_can_signals().at(robast_can_msgs::can_signal::id::e_drawer_task::DRAWER_SPEED).get_data(),
      msg.get_can_signals().at(robast_can_msgs::can_signal::id::e_drawer_task::DRAWER_STALL_GUARD_VALUE).get_data());
  }

}   // namespace utils