#include "drawer_gate/can_encoder_decoder.hpp"


std::optional<robast_can_msgs::CanMessage> CanEncoderDecoder::decode_msg(can_msgs::msg::Frame msg)
{
    return robast_can_msgs::decode_can_message(msg.id, msg.data.begin(), msg.dlc, this->can_db.can_messages);
}

can_msgs::msg::Frame CanEncoderDecoder::encode_msg(robast_can_msgs::CanMessage msg)
{
    robast_can_msgs::CanFrame can_frame = encode_can_message_into_can_frame(msg, this->can_db.can_messages);
    
    can_msgs::msg::Frame frame = can_msgs::msg::Frame();
    frame.id = can_frame.get_id();
    frame.dlc = can_frame.get_dlc();
    memcpy((void*)frame.data.begin(), (void*)can_frame.get_data(), frame.dlc);
    
    return frame;
}