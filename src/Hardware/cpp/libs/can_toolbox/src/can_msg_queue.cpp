#include "can_toolbox/can_msg_queue.hpp"

namespace drawer_controller
{
  CanMsgQueue::CanMsgQueue()
  {
    _msg_queue.clear();
    _head_of_msg_queue = 0;
  }

  void CanMsgQueue::add_element_to_msg_queue(robast_can_msgs::CanMessage msg)
  {
    _msg_queue.push_back(msg);
  }

  std::optional<robast_can_msgs::CanMessage> CanMsgQueue::get_element_from_msg_queue()
  {
    uint8_t num_of_msgs_in_queue = _msg_queue.size();
    if (num_of_msgs_in_queue == 0)
    {
      return std::nullopt;
    }
    // TODO@Jacob: This can probably be removed once we are 100% sure that the queue is not infinetly growing
    debug_printf("get_element_from_msg_queue! num_of_msgs_in_queue = %d, _msg_queue.capacity() = %d\n",
                 num_of_msgs_in_queue,
                 _msg_queue.capacity());

    if (_head_of_msg_queue == (num_of_msgs_in_queue - 1))
    {
      robast_can_msgs::CanMessage can_msg = _msg_queue[_head_of_msg_queue];
      _msg_queue.clear();
      _head_of_msg_queue = 0;
      return can_msg;
    }
    else
    {
      return _msg_queue[_head_of_msg_queue++];
    }
  }
} // namespace drawer_controller