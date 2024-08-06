#ifndef DRAWER_CONTROLLER_CONFIG_MANAGER_HPP
#define DRAWER_CONTROLLER_CONFIG_MANAGER_HPP

#include <Arduino.h>

#include <bit>
#include <memory>

#include "drawer/electrical_drawer_config.hpp"
#include "module_config/module_config_defines.hpp"
#include "motor/enconder_config.hpp"
#include "motor/motor_config.hpp"
#include "motor/motor_monitor_config.hpp"

namespace drawer_controller
{
  class ConfigManager
  {
   public:
    ConfigManager(const std::shared_ptr<ElectricalDrawerConfig> drawer_config,
                  const std::shared_ptr<EncoderConfig> encoder_config,
                  const std::shared_ptr<MotorConfig> motor_config,
                  const std::shared_ptr<MotorMonitorConfig> motor_monitor_config);

    void set_config(const uint8_t config_id, const uint32_t config_value);

   private:
    std::shared_ptr<ElectricalDrawerConfig> _drawer_config;
    std::shared_ptr<EncoderConfig> _encoder_config;
    std::shared_ptr<MotorConfig> _motor_config;
    std::shared_ptr<MotorMonitorConfig> _motor_monitor_config;

    void set_default_configs();

    void set_default_drawer_config();

    void set_default_encoder_config();

    void set_motor_config();

    void set_default_motor_monitor_config();
  };

}   // namespace drawer_controller

#endif   // DRAWER_CONTROLLER_CONFIG_MANAGER_HPP