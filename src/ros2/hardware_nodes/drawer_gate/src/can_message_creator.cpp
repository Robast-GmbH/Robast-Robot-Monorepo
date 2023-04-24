#include "drawer_gate/can_message_creator.hpp"


 CanMessageCreator::CanMessage CanMessageCreator::create_can_msg_drawer_lock(const DrawerAddress& msg,
    uint8_t can_data_open_lock) const
  {
    robast_can_msgs::CanMessage can_msg_drawer_lock = can_db_.can_messages.at(CAN_MSG_DRAWER_LOCK);

    std::vector<robast_can_msgs::CanSignal> can_signals_drawer_lock = can_msg_drawer_lock.get_can_signals();

    can_signals_drawer_lock.at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).set_data(msg.drawer_controller_id);

    // Default state is lock close
    can_signals_drawer_lock.at(CAN_SIGNAL_OPEN_LOCK_1).set_data(CAN_DATA_CLOSE_LOCK);
    can_signals_drawer_lock.at(CAN_SIGNAL_OPEN_LOCK_2).set_data(CAN_DATA_CLOSE_LOCK);

    if (msg.drawer_id == 1)
    {
      can_signals_drawer_lock.at(CAN_SIGNAL_OPEN_LOCK_1).set_data(can_data_open_lock);
    }
    if (msg.drawer_id == 2)
    {
      can_signals_drawer_lock.at(CAN_SIGNAL_OPEN_LOCK_2).set_data(can_data_open_lock);
    }

    can_msg_drawer_lock.set_can_signals(can_signals_drawer_lock);

    return can_encoder_decoder_.encode_msg(can_msg_drawer_lock);
 }

   CanMessageCreator::CanMessage CanMessageCreator::create_can_msg_drawer_led(const DrawerLeds& msg) const
  {
    robast_can_msgs::CanMessage can_msg_drawer_led = can_db_.can_messages.at(CAN_MSG_DRAWER_LED);

    std::vector<robast_can_msgs::CanSignal> can_signals_drawer_led = can_msg_drawer_led.get_can_signals();

    can_signals_drawer_led.at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).set_data(msg.drawer_address.drawer_controller_id);

    can_signals_drawer_led.at(CAN_SIGNAL_LED_RED).set_data(msg.red);
    can_signals_drawer_led.at(CAN_SIGNAL_LED_GREEN).set_data(msg.green);
    can_signals_drawer_led.at(CAN_SIGNAL_LED_BLUE).set_data(msg.blue);
    can_signals_drawer_led.at(CAN_SIGNAL_LED_BRIGHTNESS).set_data(msg.brightness);
    can_signals_drawer_led.at(CAN_SIGNAL_LED_MODE).set_data(msg.mode);

    can_msg_drawer_led.set_can_signals(can_signals_drawer_led);

    return can_encoder_decoder_.encode_msg(can_msg_drawer_led);
   }

   CanMessageCreator::CanMessage CanMessageCreator::create_can_msg_drawer_task(const DrawerTask& msg) const
   {
       robast_can_msgs::CanMessage can_msg_drawer_task = can_db_.can_messages.at(CAN_MSG_DRAWER_TASK);

       std::vector<robast_can_msgs::CanSignal> can_signals_drawer_task = can_msg_drawer_task.get_can_signals();

       can_signals_drawer_task.at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).set_data(msg.drawer_address.drawer_controller_id);

       can_signals_drawer_task.at(CAN_SIGNAL_DRAWER_GOTO_POSITION).set_data(msg.goto_position);
       can_signals_drawer_task.at(CAN_SIGNAL_DRAWER_SPEED_MODE).set_data(msg.speed_mode);
       can_signals_drawer_task.at(CAN_SIGNAL_DRAWER_STALL_GUARD_ENABLE).set_data(msg.stall_guard_enable);

       can_msg_drawer_task.set_can_signals(can_signals_drawer_task);

       return can_encoder_decoder_.encode_msg(can_msg_drawer_task);
   }