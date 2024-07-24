#ifndef MODULE_CONFIG__MODULE_CONFIG_DEFINES_HPP
#define MODULE_CONFIG__MODULE_CONFIG_DEFINES_HPP

#include <cstdint>

namespace module_config
{

    // Define the mapping struct
    template <int ID>
    struct ModuleConfigDataType;
    // Usage example:
    // using DataType = ModuleConfigDataType<MODULE_CONFIG_ID_DRAWER_MAX_SPEED>::type;
    // DataType value = 10;

    /********************************************************************************************************
     * General E-Drawer Movement configs
     *********************************************************************************************************/

#define MODULE_CONFIG_ID_DRAWER_MAX_SPEED 1 // max speed of the drawer
#define MODULE_CONFIG_ID_DRAWER_HOMING_SPEED 2
#define MODULE_CONFIG_ID_DRAWER_INITIAL_HOMING_SPEED 3
#define MODULE_CONFIG_ID_DRAWER_MOVING_IN_DECELERATION_DISTANCE 4  // distance to the target position to start deceleration (max 255)
#define MODULE_CONFIG_ID_DRAWER_MOVING_IN_FINAL_HOMING_DISTANCE 5  // the end of the distance when moving in where speed is super slow ()
#define MODULE_CONFIG_ID_DRAWER_MOVING_OUT_DECELERATION_DISTANCE 6 // distance to the target position to start deceleration (max 255)

    template <>
    struct ModuleConfigDataType<MODULE_CONFIG_ID_DRAWER_MAX_SPEED>
    {
        using type = uint32_t;
    };

    template <>
    struct ModuleConfigDataType<MODULE_CONFIG_ID_DRAWER_HOMING_SPEED>
    {
        using type = uint32_t;
    };

    template <>
    struct ModuleConfigDataType<MODULE_CONFIG_ID_DRAWER_INITIAL_HOMING_SPEED>
    {
        using type = uint32_t;
    };

    template <>
    struct ModuleConfigDataType<MODULE_CONFIG_ID_DRAWER_MOVING_IN_DECELERATION_DISTANCE>
    {
        using type = uint8_t;
    };

    template <>
    struct ModuleConfigDataType<MODULE_CONFIG_ID_DRAWER_MOVING_IN_FINAL_HOMING_DISTANCE>
    {
        using type = uint8_t;
    };

    template <>
    struct ModuleConfigDataType<MODULE_CONFIG_ID_DRAWER_MOVING_OUT_DECELERATION_DISTANCE>
    {
        using type = uint8_t;
    };

    /********************************************************************************************************
     * Configs for drawer push in auto close
     *********************************************************************************************************/

#define MODULE_CONFIG_ID_DRAWER_PUSH_IN_AUTO_CLOSE_SPEED 10
#define MODULE_CONFIG_ID_DRAWER_PUSH_IN_AUTO_CLOSE_STALL_GUARD_VALUE 11 // the higher the value the more sensitive the stall guard is
#define MODULE_CONFIG_ID_DRAWER_PUSH_IN_THRESHOLD_IN_PERCENT_OF_MAX_EXTENT 12
#define MODULE_CONFIG_ID_DRAWER_PUSH_IN_ENCODER_CHECK_INTERVAL_MS 13

    template <>
    struct ModuleConfigDataType<MODULE_CONFIG_ID_DRAWER_PUSH_IN_AUTO_CLOSE_SPEED>
    {
        using type = uint8_t;
    };

    template <>
    struct ModuleConfigDataType<MODULE_CONFIG_ID_DRAWER_PUSH_IN_AUTO_CLOSE_STALL_GUARD_VALUE>
    {
        using type = uint8_t;
    };

    template <>
    struct ModuleConfigDataType<MODULE_CONFIG_ID_DRAWER_PUSH_IN_THRESHOLD_IN_PERCENT_OF_MAX_EXTENT>
    {
        using type = float;
    };

    template <>
    struct ModuleConfigDataType<MODULE_CONFIG_ID_DRAWER_PUSH_IN_ENCODER_CHECK_INTERVAL_MS>
    {
        using type = uint32_t;
    };

} // namespace module_config

#endif // MODULE_CONFIG__MODULE_CONFIG_DEFINES_HPP