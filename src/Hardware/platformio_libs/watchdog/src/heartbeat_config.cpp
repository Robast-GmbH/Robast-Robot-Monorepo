#include "watchdog/heartbeat_config.hpp"

namespace watchdog
{
  void HeartbeatConfig::print_all_configs() const
  {
    // clang-format off
    debug_printf_color(ANSI_COLOR_BLUE, "[HeartbeatConfig]: Heartbeat interval in ms: %u\n", _heartbeat_interval_in_ms);
    // clang-format on
  }

  void HeartbeatConfig::set_heartbeat_interval_in_ms(const uint16_t heartbeat_interval_in_ms)
  {
    _heartbeat_interval_in_ms = heartbeat_interval_in_ms;
  }

  uint16_t HeartbeatConfig::get_heartbeat_interval_in_ms() const
  {
    return _heartbeat_interval_in_ms;
  }

}   // namespace watchdog