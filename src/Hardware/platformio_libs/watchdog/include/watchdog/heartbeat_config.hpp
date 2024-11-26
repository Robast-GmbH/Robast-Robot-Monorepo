#ifndef WATCHDOG_HEARTBEAT_CONFIG_HPP
#define WATCHDOG_HEARTBEAT_CONFIG_HPP

#include <cstdint>

#include "debug/debug.hpp"

namespace watchdog
{
  class HeartbeatConfig
  {
   public:
    HeartbeatConfig() = default;

    void print_all_configs() const;

    void set_heartbeat_interval_in_ms(const uint16_t heartbeat_interval_in_ms);

    uint16_t get_heartbeat_interval_in_ms() const;

   private:
    uint16_t _heartbeat_interval_in_ms;
  };

}   // namespace watchdog

#endif   // WATCHDOG_HEARTBEAT_CONFIG_HPP