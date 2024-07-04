#ifndef DRAWER_CONTROLLER_CAN_QUEUE_HPP
#define DRAWER_CONTROLLER_CAN_QUEUE_HPP

#include <memory>
#include <optional>

#include "can/can_message.h"
#include "debug/debug.hpp"

namespace drawer_controller
{
  class CanMsgQueue
  {
   public:
    CanMsgQueue();

    void add_element_to_msg_queue(robast_can_msgs::CanMessage msg);

    std::optional<robast_can_msgs::CanMessage> get_element_from_msg_queue();

   private:
    // Please mind: Normaly you would not built a queue yourself and use xQueue from FreeRTOS or std::queue
    // But: std:queue did not work and just permanently threw expections that made the ESP32 reboot
    // To use xQueue you need to use xQueueCreate to create a queue and you need to specify the size, in bytes,
    // required to hold each item in the queue, which is not possible for the CanMessage class because it contains a
    // vector of CanSignals which has a different length depending on the CanMessage. Therefore we built a queue
    // with a vector, which should be fine in this case as the queue usually only contains one or two feedback
    // messages and is rarely used. Furthermore we try to keep it as efficient as possible and try to follow what is
    // explained here: https://youtu.be/fHNmRkzxHWs?t=2541
    std::vector<robast_can_msgs::CanMessage> _msg_queue;
    uint8_t _head_of_msg_queue;
  };

}   // namespace drawer_controller

#endif   // DRAWER_CONTROLLER_CAN_QUEUE_HPP