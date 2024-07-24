#include "utils/config_manager.hpp"

namespace drawer_controller
{
  ConfigManager::ConfigManager(const std::shared_ptr<ElectricalDrawerConfigs> drawer_configs)
      : _drawer_configs{drawer_configs}
  {
    set_default_configs();
  }

  void ConfigManager::set_config(const uint8_t config_id, const uint32_t config_value)
  {
    switch (config_id)
    {
      case MODULE_CONFIG_ID_DRAWER_MAX_SPEED:
        _drawer_configs->set_drawer_max_speed(
          reinterpret_cast<module_config::ModuleConfigDataType<MODULE_CONFIG_ID_DRAWER_MAX_SPEED>::type>(config_value));
        break;

      case MODULE_CONFIG_ID_DRAWER_HOMING_SPEED:
        _drawer_configs->set_drawer_homing_speed(
          reinterpret_cast<module_config::ModuleConfigDataType<MODULE_CONFIG_ID_DRAWER_HOMING_SPEED>::type>(
            config_value));
        break;

      case MODULE_CONFIG_ID_DRAWER_INITIAL_HOMING_SPEED:
        _drawer_configs->set_drawer_initial_homing_speed(
          reinterpret_cast<module_config::ModuleConfigDataType<MODULE_CONFIG_ID_DRAWER_INITIAL_HOMING_SPEED>::type>(
            config_value));
        break;

      case MODULE_CONFIG_ID_DRAWER_MOVING_IN_DECELERATION_DISTANCE:
        _drawer_configs->set_drawer_moving_in_deceleration_distance(
          static_cast<
            module_config::ModuleConfigDataType<MODULE_CONFIG_ID_DRAWER_MOVING_IN_DECELERATION_DISTANCE>::type>(
            config_value));
        break;

      case MODULE_CONFIG_ID_DRAWER_MOVING_IN_FINAL_HOMING_DISTANCE:
        _drawer_configs->set_drawer_moving_in_final_homing_distance(
          static_cast<
            module_config::ModuleConfigDataType<MODULE_CONFIG_ID_DRAWER_MOVING_IN_FINAL_HOMING_DISTANCE>::type>(
            config_value));
        break;

      case MODULE_CONFIG_ID_DRAWER_MOVING_OUT_DECELERATION_DISTANCE:
        _drawer_configs->set_drawer_moving_out_deceleration_distance(
          static_cast<
            module_config::ModuleConfigDataType<MODULE_CONFIG_ID_DRAWER_MOVING_OUT_DECELERATION_DISTANCE>::type>(
            config_value));
        break;

      case MODULE_CONFIG_ID_DRAWER_PUSH_IN_AUTO_CLOSE_SPEED:
        _drawer_configs->set_drawer_push_in_auto_close_speed(
          static_cast<module_config::ModuleConfigDataType<MODULE_CONFIG_ID_DRAWER_PUSH_IN_AUTO_CLOSE_SPEED>::type>(
            config_value));
        break;

      case MODULE_CONFIG_ID_DRAWER_PUSH_IN_AUTO_CLOSE_STALL_GUARD_VALUE:
        _drawer_configs->set_drawer_push_in_auto_close_stall_guard_value(
          static_cast<
            module_config::ModuleConfigDataType<MODULE_CONFIG_ID_DRAWER_PUSH_IN_AUTO_CLOSE_STALL_GUARD_VALUE>::type>(
            config_value));
        break;

      case MODULE_CONFIG_ID_DRAWER_PUSH_IN_THRESHOLD_IN_PERCENT_OF_MAX_EXTENT:
        _drawer_configs->set_drawer_push_in_threshold_in_percent_of_max_extent(
          std::bit_cast<module_config::ModuleConfigDataType<
            MODULE_CONFIG_ID_DRAWER_PUSH_IN_THRESHOLD_IN_PERCENT_OF_MAX_EXTENT>::type>(config_value));

        break;

      case MODULE_CONFIG_ID_DRAWER_PUSH_IN_ENCODER_CHECK_INTERVAL_MS:
        _drawer_configs->set_drawer_push_in_encoder_check_interval_ms(
          reinterpret_cast<
            module_config::ModuleConfigDataType<MODULE_CONFIG_ID_DRAWER_PUSH_IN_ENCODER_CHECK_INTERVAL_MS>::type>(
            config_value));
        break;

      default:
        break;
    }
  }

  void ConfigManager::set_default_configs()
  {
    set_default_drawer_configs();
  }

  void ConfigManager::set_default_drawer_configs()
  {
    // go trough module_config_id_to_default_value map and set default values
    set_config(MODULE_CONFIG_ID_DRAWER_MAX_SPEED,
               module_config::module_config_id_to_default_value.at(MODULE_CONFIG_ID_DRAWER_MAX_SPEED));
    set_config(MODULE_CONFIG_ID_DRAWER_HOMING_SPEED,
               module_config::module_config_id_to_default_value.at(MODULE_CONFIG_ID_DRAWER_HOMING_SPEED));
    set_config(MODULE_CONFIG_ID_DRAWER_INITIAL_HOMING_SPEED,
               module_config::module_config_id_to_default_value.at(MODULE_CONFIG_ID_DRAWER_INITIAL_HOMING_SPEED));
    set_config(
      MODULE_CONFIG_ID_DRAWER_MOVING_IN_DECELERATION_DISTANCE,
      module_config::module_config_id_to_default_value.at(MODULE_CONFIG_ID_DRAWER_MOVING_IN_DECELERATION_DISTANCE));
    set_config(
      MODULE_CONFIG_ID_DRAWER_MOVING_IN_FINAL_HOMING_DISTANCE,
      module_config::module_config_id_to_default_value.at(MODULE_CONFIG_ID_DRAWER_MOVING_IN_FINAL_HOMING_DISTANCE));
    set_config(
      MODULE_CONFIG_ID_DRAWER_MOVING_OUT_DECELERATION_DISTANCE,
      module_config::module_config_id_to_default_value.at(MODULE_CONFIG_ID_DRAWER_MOVING_OUT_DECELERATION_DISTANCE));
    set_config(MODULE_CONFIG_ID_DRAWER_PUSH_IN_AUTO_CLOSE_SPEED,
               module_config::module_config_id_to_default_value.at(MODULE_CONFIG_ID_DRAWER_PUSH_IN_AUTO_CLOSE_SPEED));
    set_config(MODULE_CONFIG_ID_DRAWER_PUSH_IN_AUTO_CLOSE_STALL_GUARD_VALUE,
               module_config::module_config_id_to_default_value.at(
                 MODULE_CONFIG_ID_DRAWER_PUSH_IN_AUTO_CLOSE_STALL_GUARD_VALUE));
    set_config(MODULE_CONFIG_ID_DRAWER_PUSH_IN_THRESHOLD_IN_PERCENT_OF_MAX_EXTENT,
               module_config::module_config_id_to_default_value.at(
                 MODULE_CONFIG_ID_DRAWER_PUSH_IN_THRESHOLD_IN_PERCENT_OF_MAX_EXTENT));
    set_config(
      MODULE_CONFIG_ID_DRAWER_PUSH_IN_ENCODER_CHECK_INTERVAL_MS,
      module_config::module_config_id_to_default_value.at(MODULE_CONFIG_ID_DRAWER_PUSH_IN_ENCODER_CHECK_INTERVAL_MS));
  }

}   // namespace drawer_controller