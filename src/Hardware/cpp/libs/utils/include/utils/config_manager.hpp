#ifndef DRAWER_CONTROLLER_CONFIG_MANAGER_HPP
#define DRAWER_CONTROLLER_CONFIG_MANAGER_HPP

#include <bit>
#include <memory>

#include "drawer/electrical_drawer_configs.hpp"
#include "module_config/default_values_e_drawer.hpp"
#include "module_config/module_config_defines.hpp"

namespace drawer_controller
{
  class ConfigManager
  {
   public:
    ConfigManager(const std::shared_ptr<ElectricalDrawerConfigs> drawer_configs);

    void set_config(const uint8_t config_id, const uint32_t config_value);

   private:
    std::shared_ptr<ElectricalDrawerConfigs> _drawer_configs;

    void set_default_configs();

    void set_default_drawer_configs();
  };

}   // namespace drawer_controller

#endif   // DRAWER_CONTROLLER_CONFIG_MANAGER_HPP