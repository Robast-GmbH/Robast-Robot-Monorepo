#ifndef UTILS_BASE_CONFIG_MANAGER_HPP
#define UTILS_BASE_CONFIG_MANAGER_HPP

#include <bit>
#include <memory>

#include "logging/rotating_file_handler_config.hpp"
#include "module_config/module_config_defines.hpp"
#include "watchdog/heartbeat_config.hpp"

namespace utils
{
  class BaseConfigManager
  {
   public:
    BaseConfigManager();

    virtual bool set_config(const uint8_t config_id, const uint32_t config_value) = 0;

    virtual void print_all_configs() const = 0;

    std::shared_ptr<watchdog::HeartbeatConfig> get_heartbeat_config() const;

    std::shared_ptr<logging::RotatingFileHandlerConfig> get_rotating_file_handler_config() const;

   protected:
    bool set_base_config(const uint8_t config_id, const uint32_t config_value);

    void print_base_configs() const;

   private:
    const std::shared_ptr<watchdog::HeartbeatConfig> _heartbeat_config = std::make_shared<watchdog::HeartbeatConfig>();
    const std::shared_ptr<logging::RotatingFileHandlerConfig> _rotating_file_handler_config =
      std::make_shared<logging::RotatingFileHandlerConfig>();

    void set_default_configs();

    void set_default_heartbeat_config();

    void set_default_rotating_file_handler_config();
  };

}   // namespace utils

#endif   // UTILS_BASE_CONFIG_MANAGER_HPP
