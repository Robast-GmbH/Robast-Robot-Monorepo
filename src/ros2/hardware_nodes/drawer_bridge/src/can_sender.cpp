#include "drawer_bridge/can_sender.hpp"

// TODO: The CanSender should be standalone and accessible from other nodes as well (by a ros topic for example)
// TODO: When changing to the can bus from the jetson board this should be done properly
namespace drawer_bridge
{
  CanSender::CanSender(std::shared_ptr<rclcpp::Node> node) : _node(node), _is_timer_period_increased(false)
  {
    _send_can_msgs_timer = _node->create_wall_timer(std::chrono::microseconds(TIMER_PERIOD_SEND_CAN_MSGS_IN_US),
                                                    std::bind(&CanSender::send_can_msgs_timer_callback, this));

    _can_messages_publisher = _node->create_publisher<can_msgs::msg::Frame>("to_can_bus", 10);
  }

  void CanSender::send_can_msgs_timer_callback()
  {
    if (_can_msg_queue.empty())
    {
      _send_can_msgs_timer->cancel();   // cancel the timer when queue is empty
      return;
    }
    else
    {
      _can_messages_publisher->publish(this->_can_msg_queue.front());
      _can_msg_queue.pop();   // remove first element of the queue
    }
  }

  void CanSender::add_can_message_to_queue(can_msgs::msg::Frame can_msg)
  {
    _can_messages_publisher->publish(can_msg);
    // if queue was empty, the timer has been canceled before, so start timer for timer callbacks which trigger sending
    // the ascii cmds
    bool is_queue_empty = _can_msg_queue.empty();

    _can_msg_queue.push(can_msg);

    if (is_queue_empty)
    {
      _is_timer_period_increased = false;
      _send_can_msgs_timer = _node->create_wall_timer(std::chrono::microseconds(TIMER_PERIOD_SEND_CAN_MSGS_IN_US),
                                                      std::bind(&CanSender::send_can_msgs_timer_callback, this));
      RCLCPP_INFO(_node->get_logger(),
                  "Starting wall timer with period %i us!\n ",
                  TIMER_PERIOD_SEND_CAN_MSGS_IN_US,
                  _can_msg_queue.size());
    }
    else
    {
      if (_can_msg_queue.size() > MAX_QUEUE_SIZE_BEFORE_INCREASING_TIMER_PERIOD)
      {
        // In case the queue gets to large, it can happen that some can messages wont be published to the bus properly

        // therefore we need to decrease the wall timer period in such a case
        // TODO: This cant be a long term solution
        if (!_is_timer_period_increased)
        {
          _is_timer_period_increased = true;
          // TODO: How do i just change the interval of the timer? For cancel and restart it, but this might go easier
          _send_can_msgs_timer->cancel();
          _send_can_msgs_timer =
            _node->create_wall_timer(std::chrono::microseconds(INCREASED_TIMER_PERIOD_SEND_CAN_MSGS_IN_US),
                                     std::bind(&CanSender::send_can_msgs_timer_callback, this));
          RCLCPP_INFO(_node->get_logger(),
                      "Increasing wall timer period to %i us because queue is larger then %li messages!\n ",
                      INCREASED_TIMER_PERIOD_SEND_CAN_MSGS_IN_US,
                      _can_msg_queue.size());
        }
      }
    }
  }

}   // namespace drawer_bridge
