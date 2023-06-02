#include "drawer_bridge/can_message_creator.hpp"

drawer_bridge::CanMessageCreator::CanMessage drawer_bridge::CanMessageCreator::create_can_msg_drawer_unlock(
    const DrawerAddress& msg) const
{
  robast_can_msgs::CanMessage can_msg_drawer_lock = can_db_.can_messages.at(CAN_MSG_DRAWER_UNLOCK);

  std::vector<robast_can_msgs::CanSignal> can_signals_drawer_lock = can_msg_drawer_lock.get_can_signals();

  can_signals_drawer_lock.at(CAN_SIGNAL_MODULE_ID).set_data(msg.module_id);
  can_signals_drawer_lock.at(CAN_SIGNAL_DRAWER_ID).set_data(msg.drawer_id);

  can_msg_drawer_lock.set_can_signals(can_signals_drawer_lock);

  return can_encoder_decoder_.encode_msg(can_msg_drawer_lock);
}

drawer_bridge::CanMessageCreator::CanMessage drawer_bridge::CanMessageCreator::create_can_msg_drawer_led(
    const DrawerLeds& msg) const
{
  robast_can_msgs::CanMessage can_msg_drawer_led = can_db_.can_messages.at(CAN_MSG_DRAWER_LED);

  std::vector<robast_can_msgs::CanSignal> can_signals_drawer_led = can_msg_drawer_led.get_can_signals();

  can_signals_drawer_led.at(CAN_SIGNAL_MODULE_ID).set_data(msg.drawer_address.module_id);
  can_signals_drawer_led.at(CAN_SIGNAL_DRAWER_ID).set_data(msg.drawer_address.drawer_id);

  can_signals_drawer_led.at(CAN_SIGNAL_LED_RED).set_data(msg.red);
  can_signals_drawer_led.at(CAN_SIGNAL_LED_GREEN).set_data(msg.green);
  can_signals_drawer_led.at(CAN_SIGNAL_LED_BLUE).set_data(msg.blue);
  can_signals_drawer_led.at(CAN_SIGNAL_LED_BRIGHTNESS).set_data(msg.brightness);
  can_signals_drawer_led.at(CAN_SIGNAL_LED_MODE).set_data(msg.mode);

  can_msg_drawer_led.set_can_signals(can_signals_drawer_led);

  return can_encoder_decoder_.encode_msg(can_msg_drawer_led);
}

drawer_bridge::CanMessageCreator::CanMessage drawer_bridge::CanMessageCreator::create_can_msg_drawer_task(
    const DrawerTask& msg) const
{
  robast_can_msgs::CanMessage can_msg_drawer_task = can_db_.can_messages.at(CAN_MSG_ELECTRICAL_DRAWER_TASK);

  std::vector<robast_can_msgs::CanSignal> can_signals_drawer_task = can_msg_drawer_task.get_can_signals();

  can_signals_drawer_task.at(CAN_SIGNAL_MODULE_ID).set_data(msg.drawer_address.module_id);
  can_signals_drawer_task.at(CAN_SIGNAL_DRAWER_ID).set_data(msg.drawer_address.drawer_id);

  can_signals_drawer_task.at(CAN_SIGNAL_DRAWER_TARGET_POSITION).set_data(msg.target_position);
  can_signals_drawer_task.at(CAN_SIGNAL_DRAWER_SPEED_MODE).set_data(msg.speed_mode);
  can_signals_drawer_task.at(CAN_SIGNAL_DRAWER_STALL_GUARD_ENABLE).set_data(msg.stall_guard_enable);

  can_msg_drawer_task.set_can_signals(can_signals_drawer_task);

  return can_encoder_decoder_.encode_msg(can_msg_drawer_task);
}