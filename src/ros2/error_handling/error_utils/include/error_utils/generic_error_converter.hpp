#pragma once
// TODO discuss if we want to use pragma once

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>

template <typename MsgT>
class MessageConverter
{
public:
  MessageConverter() {}

  std::string messageToString(const MsgT &msg)
  {
    std::shared_ptr<MsgT> serialized_msg = std::make_shared<MsgT>(msg);
    std::string serialized_str(reinterpret_cast<const char *>(serialized_msg.get()), sizeof(MsgT));
    serialized_msg.reset();  // Freigabe des std::shared_ptr-Objekts
    return serialized_str;
  }

  MsgT stringToMessage(const std::string &serialized_str)
  {
    MsgT deserialized_msg;
    memcpy(&deserialized_msg, serialized_str.data(), sizeof(MsgT));
    return deserialized_msg;
  }
};
