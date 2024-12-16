#ifndef WATCHDOG__HEARTBEAT_HPP
#define WATCHDOG__HEARTBEAT_HPP

#include "can_toolbox/can_utils.hpp"
#include "watchdog/heartbeat_config.hpp"

namespace watchdog
{

  class Heartbeat
  {
   public:
    Heartbeat(const uint32_t module_id,
              const std::shared_ptr<can_toolbox::CanUtils> can_utils,
              const std::shared_ptr<watchdog::HeartbeatConfig> heartbeat_config);

    void generate_heartbeat();

   private:
    const uint32_t _module_id;

    const std::shared_ptr<can_toolbox::CanUtils> _can_utils;
    const std::shared_ptr<watchdog::HeartbeatConfig> _heartbeat_config;

    uint32_t _last_heartbeat_time_in_ms = 0;
  };

}   // namespace watchdog

#endif   // WATCHDOG__HEARTBEAT_HPP