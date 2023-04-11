#include "can_msgs/msg/frame.hpp"
#include "can/can_message.h"
#include "can/can_helper.h"
#include "can/can_db.hpp"

class CanEncoderDecoder {
public:
    can_msgs::msg::Frame encode_msg(robast_can_msgs::CanMessage msg);

    std::optional<robast_can_msgs::CanMessage> decode_msg(can_msgs::msg::Frame msg);

private:
    
      robast_can_msgs::CanDb can_db = robast_can_msgs::CanDb();


};