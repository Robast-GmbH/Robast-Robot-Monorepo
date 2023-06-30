#pragma once 
#TODO discuss if we want to use pragma once 

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>

template <typename MsgT>
class MessageConverter {
public:
  MessageConverter()
    : node_(rclcpp::NodeOptions())
  {}

  std::string messageToString(const MsgT& msg) {
    auto serialized_msg = std::make_shared<MsgT>();
    serialized_msg->serialize(msg);

    std::string serialized_str(reinterpret_cast<const char*>(serialized_msg->buffer.data()),
                               serialized_msg->buffer.size());

    return serialized_str;
  }

  MsgT stringToMessage(const std::string& serialized_str) {
    auto serialized_msg = std::make_shared<MsgT>();
    serialized_msg->buffer.resize(serialized_str.size());
    std::memcpy(serialized_msg->buffer.data(), serialized_str.data(), serialized_str.size());

    MsgT deserialized_msg;
    deserialized_msg.deserialize(*serialized_msg);

    return deserialized_msg;
  }

private:
  rclcpp::Node node_;
};
