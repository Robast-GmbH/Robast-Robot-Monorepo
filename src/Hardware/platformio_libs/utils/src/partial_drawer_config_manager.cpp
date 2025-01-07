#include "utils/partial_drawer_config_manager.hpp"

namespace utils
{
  PartialDrawerConfigManager::PartialDrawerConfigManager()
  {
    set_default_configs();
  }

  bool PartialDrawerConfigManager::set_config(const uint8_t config_id, const uint32_t config_value)
  {
    switch (config_id)
    {
      case module_config::tray_manager::SPEED_DEVIATION_IN_PERCENTAGE_FOR_STALL_WHEN_CLOSING_LID:
        _tray_manager_config->set_speed_deviation_in_percentage_for_stall_when_closing_lid(
          std::bit_cast<module_config::ModuleSetting<
            module_config::tray_manager::SPEED_DEVIATION_IN_PERCENTAGE_FOR_STALL_WHEN_CLOSING_LID>::type>(
            config_value));
        break;

      case module_config::tray_manager::POSITION_OFFSET_FOR_TRAY_LID_COMPUTATION:
        _tray_manager_config->set_position_offset_for_tray_lid_computation(
          static_cast<
            module_config::ModuleSetting<module_config::tray_manager::POSITION_OFFSET_FOR_TRAY_LID_COMPUTATION>::type>(
            config_value));
        break;

      case module_config::tray_manager::DISTANCE_TO_TRAY_LID_THRESHOLD:
        _tray_manager_config->set_distance_to_tray_lid_threshold(
          static_cast<module_config::ModuleSetting<module_config::tray_manager::DISTANCE_TO_TRAY_LID_THRESHOLD>::type>(
            config_value));
        break;

      case module_config::tray_manager::TARGET_SPEED_TO_CLOSE_TRAY_LID:
        _tray_manager_config->set_target_speed_to_close_tray_lid(
          static_cast<module_config::ModuleSetting<module_config::tray_manager::TARGET_SPEED_TO_CLOSE_TRAY_LID>::type>(
            config_value));
        break;

      default:
        return EDrawerConfigManager::set_config(config_id, config_value);
    }
    return true;
  }

  void PartialDrawerConfigManager::print_all_configs() const
  {
    EDrawerConfigManager::print_e_drawer_configs();

    _tray_manager_config->print_all_configs();
  }

  std::shared_ptr<tray::TrayManagerConfig> PartialDrawerConfigManager::get_tray_manager_config() const
  {
    return _tray_manager_config;
  }

  void PartialDrawerConfigManager::set_default_configs()
  {
    set_default_tray_manager_config();
  }

  void PartialDrawerConfigManager::set_default_tray_manager_config()
  {
    this->set_config(
      module_config::tray_manager::SPEED_DEVIATION_IN_PERCENTAGE_FOR_STALL_WHEN_CLOSING_LID,
      std::bit_cast<uint32_t>(
        module_config::ModuleSetting<
          module_config::tray_manager::SPEED_DEVIATION_IN_PERCENTAGE_FOR_STALL_WHEN_CLOSING_LID>::default_value));

    this->set_config(
      module_config::tray_manager::POSITION_OFFSET_FOR_TRAY_LID_COMPUTATION,
      static_cast<uint32_t>(module_config::ModuleSetting<
                            module_config::tray_manager::POSITION_OFFSET_FOR_TRAY_LID_COMPUTATION>::default_value));

    this->set_config(
      module_config::tray_manager::DISTANCE_TO_TRAY_LID_THRESHOLD,
      static_cast<uint32_t>(
        module_config::ModuleSetting<module_config::tray_manager::DISTANCE_TO_TRAY_LID_THRESHOLD>::default_value));

    this->set_config(
      module_config::tray_manager::TARGET_SPEED_TO_CLOSE_TRAY_LID,
      static_cast<uint32_t>(
        module_config::ModuleSetting<module_config::tray_manager::TARGET_SPEED_TO_CLOSE_TRAY_LID>::default_value));
  }

}   // namespace utils
