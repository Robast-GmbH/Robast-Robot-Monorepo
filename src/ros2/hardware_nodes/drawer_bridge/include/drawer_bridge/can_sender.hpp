#ifndef DRAWER_BRIDGE__CAN_SENDER_HPP_
#define DRAWER_BRIDGE__CAN_SENDER_HPP_

#include <queue>
#include <rclcpp/rclcpp.hpp>

#include "can_msgs/msg/frame.hpp"
#include "qos_config.hpp"

#define TIMER_PERIOD_SEND_CAN_MSGS_IN_US 500   // this seems to be about the lowest period possible (200 does not work)

namespace drawer_bridge
{
  class CanSender
  {
   public:
    CanSender(std::shared_ptr<rclcpp::Node> node);

    void add_can_message_to_queue(can_msgs::msg::Frame can_msg);

   private:
    std::shared_ptr<rclcpp::Node> _node;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr _can_messages_publisher;
    rclcpp::TimerBase::SharedPtr _send_can_msgs_timer;

    std::queue<can_msgs::msg::Frame> _can_msg_queue;

    QoSConfig _qos_config = QoSConfig();

    void send_can_msgs_timer_callback();
  };
}   // namespace drawer_bridge

#endif   // DRAWER_BRIDGE__CAN_SENDER_HPP_