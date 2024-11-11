#ifndef UTILS_CONFIG_MANAGER_HPP
#define UTILS_CONFIG_MANAGER_HPP

#include <bit>
#include <memory>

#include "drawer/electrical_drawer_config.hpp"
#include "module_config/module_config_defines.hpp"
#include "motor/enconder_config.hpp"
#include "motor/motor_config.hpp"
#include "motor/motor_monitor_config.hpp"
#include "tray/tray_manager_config.hpp"
#include "watchdog/heartbeat_config.hpp"

namespace utils
{
  class ConfigManager
  {
   public:
    ConfigManager(const std::shared_ptr<drawer::ElectricalDrawerConfig> drawer_config,
                  const std::shared_ptr<motor::EncoderConfig> encoder_config,
                  const std::shared_ptr<motor::MotorConfig> motor_config,
                  const std::shared_ptr<motor::MotorMonitorConfig> motor_monitor_config,
                  const std::shared_ptr<tray::TrayManagerConfig> tray_manager_config,
                  const std::shared_ptr<watchdog::HeartbeatConfig> heartbeat_config);

    bool set_config(const uint8_t config_id, const uint32_t config_value);

   private:
    const std::shared_ptr<drawer::ElectricalDrawerConfig> _drawer_config;
    const std::shared_ptr<motor::EncoderConfig> _encoder_config;
    const std::shared_ptr<motor::MotorConfig> _motor_config;
    const std::shared_ptr<motor::MotorMonitorConfig> _motor_monitor_config;
    const std::shared_ptr<tray::TrayManagerConfig> _tray_manager_config;
    const std::shared_ptr<watchdog::HeartbeatConfig> _heartbeat_config;

    void set_default_configs();

    void set_default_drawer_config();

    void set_default_encoder_config();

    void set_default_motor_config();

    void set_default_motor_monitor_config();

    void set_default_tray_manager_config();

    void set_default_heartbeat_config();
  };

}   // namespace utils

#endif   // UTILS_CONFIG_MANAGER_HPP