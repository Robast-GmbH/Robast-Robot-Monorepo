#ifndef MODULE_CONFIG__DEFAULT_VALUES_ENCODER_HPP
#define MODULE_CONFIG__DEFAULT_VALUES_ENCODER_HPP

#include <cstdint>
#include <unordered_map>

#include "module_config/module_config_defines.hpp"

namespace module_config_encoder
{
    const std::unordered_map<uint8_t, uint32_t> module_config_id_to_default_value = {
        {MODULE_CONFIG_ID_OPEN_LOOP_COUNT_DRAWER_MAX_EXTENT, 85000},
        {MODULE_CONFIG_ID_ENCODER_COUNT_DRAWER_MAX_EXTENT, 86000},
        {MODULE_CONFIG_ID_DRAWER_POSITION_OPEN_LOOP_INTEGRAL_GAIN, 1000},
        {MODULE_CONFIG_ID_DRAWER_PUSH_IN_THRESHOLD_IN_PERCENT_OF_MAX_EXTENT, 0.005},
        {MODULE_CONFIG_ID_DRAWER_PUSH_IN_ENCODER_CHECK_INTERVAL_MS, 200}};
} // namespace module_config_encoder

#endif // MODULE_CONFIG__DEFAULT_VALUES_ENCODER_HPP