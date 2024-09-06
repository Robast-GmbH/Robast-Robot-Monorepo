#ifndef CONFIG_USER_CONFIG_HPP
#define CONFIG_USER_CONFIG_HPP

#include "module_id/module_id.hpp"
#include "switch/switch.hpp"

namespace config
{
  struct UserConfig
  {
    const char* module_version;
    module_id::ModulePrefix module_prefix;
    uint32_t unique_module_id;
    uint8_t lock_id;
    bool is_shaft_direction_inverted;
    switch_lib::Switch::SwitchType endstop_switch_type;
    bool use_color_fade;
  };
}   // namespace config

#endif   // CONFIG_USER_CONFIG_HPP