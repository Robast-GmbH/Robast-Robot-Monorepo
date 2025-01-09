#ifndef ERROR_UTILS__GENERIC_ERROR_CONVERTER_HPP_
#define ERROR_UTILS__GENERIC_ERROR_CONVERTER_HPP_

#include <cstring>
#include <memory>
#include <sstream>
#include <string>

namespace error_utils
{
  template <typename msg_type>
  std::string message_to_string(const msg_type &msg)
  {
    std::ostringstream oss;
    oss.write(reinterpret_cast<const char *>(&msg), sizeof(msg_type));
    return oss.str();
  }

  template <typename msg_type>
  msg_type string_to_message(const std::string &serialized_str)
  {
    msg_type deserialized_msg;
    std::istringstream iss(serialized_str);
    iss.read(reinterpret_cast<char *>(&deserialized_msg), sizeof(msg_type));
    return deserialized_msg;
  }
}   // namespace error_utils

#endif   // ERROR_UTILS__GENERIC_ERROR_CONVERTER_HPP_
