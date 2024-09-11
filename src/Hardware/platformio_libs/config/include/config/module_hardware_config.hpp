#ifndef CONFIG_MODULE_HARDWARE_CONFIG_HPP
#define CONFIG_MODULE_HARDWARE_CONFIG_HPP

#include <cstdint>
#include <cstring>

#include "module_id/module_id.hpp"

namespace config
{
  namespace version
  {
    constexpr const char ROSAH[] = "Rosah";
    constexpr const char CURA[] = "Cura";
  }   // namespace version

  struct ModuleHardwareConfig
  {
    bool module_contains_a_drawer;
    bool is_electrical_drawer;
    bool use_encoder;
    uint8_t num_of_leds;
    module_id::ModulePrefix module_prefix;

    // Parameterized Constructor
    constexpr ModuleHardwareConfig(const bool module_contains_a_drawer_arg,
                                   const bool is_electrical_drawer_arg,
                                   const bool use_encoder_arg,
                                   const uint8_t num_of_leds_arg,
                                   const module_id::ModulePrefix module_prefix_arg)
        : module_contains_a_drawer(module_contains_a_drawer_arg),
          is_electrical_drawer(is_electrical_drawer_arg),
          use_encoder(use_encoder_arg),
          num_of_leds(num_of_leds_arg),
          module_prefix(module_prefix_arg)
    {
    }
  };

  constexpr bool MODULE_CONTAINS_A_DRAWER = true;
  constexpr bool MODULE_DOES_NOT_CONTAIN_A_DRAWER = false;
  constexpr bool IS_ELECTRICAL_DRAWER = true;
  constexpr bool IS_MANUAL_DRAWER = false;
  constexpr bool USE_ENCODER = true;
  constexpr bool USE_NO_ENCODER = false;

  template <const char* version>
  constexpr ModuleHardwareConfig get_module_hardware_config(module_id::ModulePrefix module_prefix)
  {
    // Check if the version is supported at compile-time
    constexpr bool is_supported_version =
      (std::strcmp(version, version::ROSAH) == 0) || (std::strcmp(version, version::CURA) == 0);
    static_assert(is_supported_version, "Unsupported module hardware version at compile-time.");

    uint8_t NUM_OF_LEDS_DRAWER = 0;
    uint8_t NUM_OF_LEDS_BASE = 0;

    if constexpr (std::strcmp(version, version::CURA) == 0)
    {
      NUM_OF_LEDS_DRAWER = 21;
      NUM_OF_LEDS_BASE = 119;
    }
    else if constexpr (std::strcmp(version, version::ROSAH) == 0)
    {
      NUM_OF_LEDS_DRAWER = 18;
      NUM_OF_LEDS_BASE = 120;
    }

    // Handle the different module prefixes
    switch (module_prefix)
    {
      using enum module_id::ModulePrefix;
      case BASE_LED_CONTROLLER:
        return ModuleHardwareConfig(
          MODULE_DOES_NOT_CONTAIN_A_DRAWER, IS_MANUAL_DRAWER, USE_NO_ENCODER, NUM_OF_LEDS_BASE, BASE_LED_CONTROLLER);
      case MANUAL_DRAWER_10x40:
        return ModuleHardwareConfig(
          MODULE_CONTAINS_A_DRAWER, IS_MANUAL_DRAWER, USE_NO_ENCODER, NUM_OF_LEDS_DRAWER, MANUAL_DRAWER_10x40);
      case MANUAL_DRAWER_20x40:
        return ModuleHardwareConfig(
          MODULE_CONTAINS_A_DRAWER, IS_MANUAL_DRAWER, USE_NO_ENCODER, NUM_OF_LEDS_DRAWER, MANUAL_DRAWER_20x40);
      case MANUAL_DRAWER_30x40:
        return ModuleHardwareConfig(
          MODULE_CONTAINS_A_DRAWER, IS_MANUAL_DRAWER, USE_NO_ENCODER, NUM_OF_LEDS_DRAWER, MANUAL_DRAWER_30x40);
      case E_DRAWER_10x40:
        return ModuleHardwareConfig(
          MODULE_CONTAINS_A_DRAWER, IS_ELECTRICAL_DRAWER, USE_ENCODER, NUM_OF_LEDS_DRAWER, E_DRAWER_10x40);
      case PARTIAL_DRAWER_10x40x8:
        return ModuleHardwareConfig(
          MODULE_CONTAINS_A_DRAWER, IS_ELECTRICAL_DRAWER, USE_ENCODER, NUM_OF_LEDS_DRAWER, PARTIAL_DRAWER_10x40x8);
    }
  };

}   // namespace config

#endif   // CONFIG_MODULE_HARDWARE_CONFIG_HPP
