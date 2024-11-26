#include "tray/tray_manager_config.hpp"

namespace tray
{
  void TrayManagerConfig::print_all_configs() const
  {
    // clang-format off
    debug_printf_color(ANSI_COLOR_BLUE, "[TrayManagerConfig]: Speed deviation in percentage for stall when closing lid: %.2f\n", _speed_deviation_in_percentage_for_stall_when_closing_lid);
    debug_printf_color(ANSI_COLOR_BLUE, "[TrayManagerConfig]: Position offset for tray lid computation: %u\n", _position_offset_for_tray_lid_computation);
    debug_printf_color(ANSI_COLOR_BLUE, "[TrayManagerConfig]: Distance to tray lid threshold: %u\n", _distance_to_tray_lid_threshold);
    debug_printf_color(ANSI_COLOR_BLUE, "[TrayManagerConfig]: Target speed to close tray lid: %u\n", _target_speed_to_close_tray_lid);
    // clang-format on
  }

  void TrayManagerConfig::set_speed_deviation_in_percentage_for_stall_when_closing_lid(
    const float speed_deviation_in_percentage_for_stall_when_closing_lid)
  {
    _speed_deviation_in_percentage_for_stall_when_closing_lid =
      speed_deviation_in_percentage_for_stall_when_closing_lid;
  }

  void TrayManagerConfig::set_position_offset_for_tray_lid_computation(
    const uint8_t position_offset_for_tray_lid_computation)
  {
    _position_offset_for_tray_lid_computation = position_offset_for_tray_lid_computation;
  }

  void TrayManagerConfig::set_distance_to_tray_lid_threshold(const uint8_t distance_to_tray_lid_threshold)
  {
    _distance_to_tray_lid_threshold = distance_to_tray_lid_threshold;
  }

  void TrayManagerConfig::set_target_speed_to_close_tray_lid(const uint8_t target_speed_to_close_tray_lid)
  {
    _target_speed_to_close_tray_lid = target_speed_to_close_tray_lid;
  }

  float TrayManagerConfig::get_speed_deviation_in_percentage_for_stall_when_closing_lid() const
  {
    return _speed_deviation_in_percentage_for_stall_when_closing_lid;
  }

  uint8_t TrayManagerConfig::get_position_offset_for_tray_lid_computation() const
  {
    return _position_offset_for_tray_lid_computation;
  }

  uint8_t TrayManagerConfig::get_distance_to_tray_lid_threshold() const
  {
    return _distance_to_tray_lid_threshold;
  }

  uint8_t TrayManagerConfig::get_target_speed_to_close_tray_lid() const
  {
    return _target_speed_to_close_tray_lid;
  }
}   // namespace tray