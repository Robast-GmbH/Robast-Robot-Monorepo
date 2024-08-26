#ifndef DRAWER_BRIDGE__CAN_ENCODER_DECODER_HPP_
#define DRAWER_BRIDGE__CAN_ENCODER_DECODER_HPP_

#include "can/can_db.hpp"
#include "can/can_helper.hpp"
#include "can/can_message.hpp"
#include "can_msgs/msg/frame.hpp"

namespace drawer_bridge
{
  class CanEncoderDecoder
  {
   public:
    can_msgs::msg::Frame encode_msg(robast_can_msgs::CanMessage msg) const;

    std::optional<robast_can_msgs::CanMessage> decode_msg(can_msgs::msg::Frame msg) const;

   private:
    robast_can_msgs::CanDb _can_db = robast_can_msgs::CanDb();
  };
}   // namespace drawer_bridge

#endif