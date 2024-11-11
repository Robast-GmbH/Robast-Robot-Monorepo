#include "watchdog/heartbeat.hpp"

namespace watchdog
{
  Heartbeat::Heartbeat(const uint32_t module_id,
                       const std::shared_ptr<can_toolbox::CanUtils> can_utils,
                       const std::shared_ptr<watchdog::HeartbeatConfig> heartbeat_config)
      : _module_id{module_id}, _can_utils{can_utils}, _heartbeat_config{heartbeat_config}
  {
  }

  void Heartbeat::generate_heartbeat()
  {
    const uint32_t current_time = millis();
    const uint32_t elapsed_time = current_time - _last_heartbeat_time_in_ms;

    if (elapsed_time >= _heartbeat_config->get_heartbeat_interval_in_ms())
    {
      _can_utils->enqueue_heartbeat_msg(_module_id);
      _last_heartbeat_time_in_ms = current_time;
    }
  }

}   // namespace watchdog