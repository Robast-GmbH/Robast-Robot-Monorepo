#ifndef WATCHDOG__HEARTBEAT_HPP
#define WATCHDOG__HEARTBEAT_HPP

#include "can_toolbox/can_utils.hpp"
#include "logging/rotating_file_handler.hpp"
#include "watchdog/heartbeat_config.hpp"

namespace watchdog
{

  class Heartbeat
  {
   public:
    Heartbeat(const uint32_t module_id,
              const std::shared_ptr<can_toolbox::CanUtils> can_utils,
              const std::shared_ptr<watchdog::HeartbeatConfig> heartbeat_config,
              const std::shared_ptr<logging::RotatingFileHandler> rotating_file_logger);

    void generate_heartbeat();

   private:
    const uint32_t _module_id;

    const std::shared_ptr<can_toolbox::CanUtils> _can_utils;
    const std::shared_ptr<watchdog::HeartbeatConfig> _heartbeat_config;
    const std::shared_ptr<logging::RotatingFileHandler> _rotating_file_logger;

    uint32_t _last_heartbeat_time_in_ms = 0;
  };

}   // namespace watchdog

#endif   // WATCHDOG__HEARTBEAT_HPP