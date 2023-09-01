#include "drawer_bridge/can_sender.hpp"

// TODO: The CanSender should be standalone and accessible from other nodes as well (by a ros topic for example)
// TODO: When changing to the can bus from the jetson board this should be done properly
namespace drawer_bridge
{
  CanSender::CanSender(std::shared_ptr<rclcpp::Node> node) : _node(node)
  {
    _send_can_msgs_timer = _node->create_wall_timer(std::chrono::milliseconds(TIMER_PERIOD_SEND_CAN_MSGS_IN_MS),
                                                    std::bind(&CanSender::send_can_msgs_timer_callback, this));

    _can_messages_publisher =
      _node->create_publisher<can_msgs::msg::Frame>("to_can_bus", _qos_config.get_qos_can_messages());
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
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%d'\n ", can_message.id); //TODO: Fix this?

      _can_messages_publisher->publish(this->_can_msg_queue.front());
      _can_msg_queue.pop();   // remove first element of the queue
    }
  }

  void CanSender::add_can_message_to_queue(can_msgs::msg::Frame can_msg)
  {
    // if queue was empty, the timer has been canceled before, so start timer for timer callbacks which trigger sending
    // the ascii cmds
    if (_can_msg_queue.empty())
    {
      _can_msg_queue.push(can_msg);
      _send_can_msgs_timer = _node->create_wall_timer(std::chrono::milliseconds(TIMER_PERIOD_SEND_CAN_MSGS_IN_MS),
                                                      std::bind(&CanSender::send_can_msgs_timer_callback, this));
    }
    else
    {
      _can_msg_queue.push(can_msg);
    }
  }

}   // namespace drawer_bridge
