#include "drawer_bridge/can_encoder_decoder.hpp"

std::optional<robast_can_msgs::CanMessage> drawer_bridge::CanEncoderDecoder::decode_msg(can_msgs::msg::Frame msg) const
{
  return robast_can_msgs::decode_can_message(msg.id, msg.data.begin(), msg.dlc, this->_can_db.can_messages);
}

can_msgs::msg::Frame drawer_bridge::CanEncoderDecoder::encode_msg(robast_can_msgs::CanMessage msg) const
{
  robast_can_msgs::CanFrame can_frame = encode_can_message_into_can_frame(msg, this->_can_db.can_messages);

  can_msgs::msg::Frame frame = can_msgs::msg::Frame();
  frame.id = can_frame.get_id();
  frame.dlc = can_frame.get_dlc();
  memcpy((void*) frame.data.begin(), (void*) can_frame.get_data(), frame.dlc);

  return frame;
}