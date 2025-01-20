#ifndef ERROR_UTILS__GENERIC_ERROR_CONVERTER_HPP_
#define ERROR_UTILS__GENERIC_ERROR_CONVERTER_HPP_

#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

namespace error_utils
{
  namespace
  {
    inline std::string to_hex_string(const std::string &input)
    {
      std::ostringstream oss;
      for (unsigned char c : input)
      {
        oss << "\\x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(c);
      }
      return oss.str();
    }

    inline std::string from_hex_string(const std::string &hex)
    {
      std::string output;
      for (size_t i = 0; i < hex.length(); i += 4)
      {
        std::string byteString = hex.substr(i + 2, 2);
        char byte = static_cast<char>(strtol(byteString.c_str(), nullptr, 16));
        output.push_back(byte);
      }
      return output;
    }
  }   // namespace

  template <typename msg_type>
  std::string message_to_string(const msg_type &msg)
  {
    std::ostringstream oss;
    oss.write(reinterpret_cast<const char *>(&msg), sizeof(msg_type));
    return to_hex_string(oss.str());
  }

  template <typename msg_type>
  msg_type string_to_message(const std::string &serialized_str)
  {
    msg_type deserialized_msg;
    std::istringstream iss(from_hex_string(serialized_str));
    iss.read(reinterpret_cast<char *>(&deserialized_msg), sizeof(msg_type));
    return deserialized_msg;
  }
}   // namespace error_utils

#endif   // ERROR_UTILS__GENERIC_ERROR_CONVERTER_HPP_
