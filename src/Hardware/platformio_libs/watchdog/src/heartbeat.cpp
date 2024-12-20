#include "watchdog/heartbeat.hpp"

namespace watchdog
{
  Heartbeat::Heartbeat(const uint32_t module_id,
                       const std::shared_ptr<can_toolbox::CanUtils> can_utils,
                       const std::shared_ptr<watchdog::HeartbeatConfig> heartbeat_config,
                       const std::shared_ptr<logging::RotatingFileHandler> rotating_file_logger)
      : _module_id{module_id},
        _can_utils{can_utils},
        _heartbeat_config{heartbeat_config},
        _rotating_file_logger{rotating_file_logger}
  {
  }

  void Heartbeat::generate_heartbeat()
  {
    const uint32_t current_time = millis();
    const uint32_t elapsed_time = current_time - _last_heartbeat_time_in_ms;
    const uint16_t heartbeat_interval_in_ms = _heartbeat_config->get_heartbeat_interval_in_ms();

    if (elapsed_time >= heartbeat_interval_in_ms)
    {
      _can_utils->enqueue_heartbeat_msg(_module_id, heartbeat_interval_in_ms);
      _rotating_file_logger->write("HB at " + std::to_string(current_time) + " ms.");
      _last_heartbeat_time_in_ms = current_time;
    }
  }

}   // namespace watchdog