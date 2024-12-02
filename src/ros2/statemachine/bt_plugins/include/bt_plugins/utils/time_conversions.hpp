#include <chrono>

#include "builtin_interfaces/msg/time.hpp"

namespace utils
{
  std::chrono::milliseconds convert_to_milliseconds(const builtin_interfaces::msg::Time &time)
  {
    return std::chrono::milliseconds(time.sec * 1000 + time.nanosec / 1000000);
  }

}   // namespace utils
