#ifndef QOS_BRIDGE_HPP
#define QOS_BRIDGE_HPP

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

template<typename MessageT>
class QosBridge : public rclcpp::Node
{
public:
  QosBridge(const std::string& input_topic, const std::string& output_topic, const rclcpp::QoS& input_qos, size_t output_queue_size)
  : Node("qos_bridge")
  {
      _subscription = this->create_subscription<MessageT>(
        input_topic, input_qos, std::bind(&QosBridge::bridge_callback, this, std::placeholders::_1));
      
      _publisher = this->create_publisher<MessageT>(output_topic, rclcpp::QoS(rclcpp::KeepLast(output_queue_size)));
  }

private:
  void bridge_callback(const typename MessageT::SharedPtr msg)
  {
    MessageT msg_copy = *msg;
    RCLCPP_INFO(this->get_logger(), "Re-Publishing message");
    _publisher->publish(msg_copy);
  }

  typename rclcpp::Publisher<MessageT>::SharedPtr _publisher;
  typename rclcpp::Subscription<MessageT>::SharedPtr _subscription;
};

#endif // QOS_BRIDGE_HPP
