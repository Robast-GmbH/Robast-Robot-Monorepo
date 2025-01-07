#include "utils/base_config_manager.hpp"

namespace utils
{
  BaseConfigManager::BaseConfigManager()
  {
    set_default_configs();
  }

  bool BaseConfigManager::set_base_config(const uint8_t config_id, const uint32_t config_value)
  {
    switch (config_id)
    {
      case module_config::watchdog::HEARTBEAT_INTERVAL_IN_MS:
        _heartbeat_config->set_heartbeat_interval_in_ms(
          static_cast<module_config::ModuleSetting<module_config::watchdog::HEARTBEAT_INTERVAL_IN_MS>::type>(
            config_value));
        break;

      case module_config::logging::ROTATING_FILE_HANDLER_MAX_FILE_SIZE_IN_BYTES:
        _rotating_file_handler_config->set_max_file_size_in_bytes(
          static_cast<
            module_config::ModuleSetting<module_config::logging::ROTATING_FILE_HANDLER_MAX_FILE_SIZE_IN_BYTES>::type>(
            config_value));
        break;

      case module_config::logging::ROTATING_FILE_HANDLER_MAX_FILES:
        _rotating_file_handler_config->set_max_files(
          static_cast<module_config::ModuleSetting<module_config::logging::ROTATING_FILE_HANDLER_MAX_FILES>::type>(
            config_value));
        break;

      default:
        return false;
    }
    return true;
  }

  void BaseConfigManager::print_base_configs() const
  {
    _heartbeat_config->print_all_configs();
    _rotating_file_handler_config->print_all_configs();
  }

  std::shared_ptr<watchdog::HeartbeatConfig> BaseConfigManager::get_heartbeat_config() const
  {
    return _heartbeat_config;
  }

  std::shared_ptr<logging::RotatingFileHandlerConfig> BaseConfigManager::get_rotating_file_handler_config() const
  {
    return _rotating_file_handler_config;
  }

  void BaseConfigManager::set_default_configs()
  {
    set_default_heartbeat_config();

    set_default_rotating_file_handler_config();
  }

  void BaseConfigManager::set_default_heartbeat_config()
  {
    set_base_config(module_config::watchdog::HEARTBEAT_INTERVAL_IN_MS,
                    static_cast<uint32_t>(
                      module_config::ModuleSetting<module_config::watchdog::HEARTBEAT_INTERVAL_IN_MS>::default_value));
  }

  void BaseConfigManager::set_default_rotating_file_handler_config()
  {
    set_base_config(
      module_config::logging::ROTATING_FILE_HANDLER_MAX_FILE_SIZE_IN_BYTES,
      static_cast<uint32_t>(module_config::ModuleSetting<
                            module_config::logging::ROTATING_FILE_HANDLER_MAX_FILE_SIZE_IN_BYTES>::default_value));
    set_base_config(
      module_config::logging::ROTATING_FILE_HANDLER_MAX_FILES,
      static_cast<uint32_t>(
        module_config::ModuleSetting<module_config::logging::ROTATING_FILE_HANDLER_MAX_FILES>::default_value));
  }

}   // namespace utils
