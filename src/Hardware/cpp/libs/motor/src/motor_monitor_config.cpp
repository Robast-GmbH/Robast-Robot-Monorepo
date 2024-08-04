#include "motor/motor_monitor_config.hpp"

namespace drawer_controller
{
  void MotorMonitorConfig::set_active_speed_threshold(const uint32_t active_speed_threshold)
  {
    _active_speed_threshold = active_speed_threshold;
  }

  void MotorMonitorConfig::set_lower_position_threshold(const uint32_t lower_position_threshold)
  {
    _lower_position_threshold = lower_position_threshold;
  }

  void MotorMonitorConfig::set_max_time_diff_between_encoder_measurements_in_ms(
    const uint32_t max_time_diff_between_encoder_measurements_in_ms)
  {
    _max_time_diff_between_encoder_measurements_in_ms = max_time_diff_between_encoder_measurements_in_ms;
  }

  void MotorMonitorConfig::set_speed_deviation_in_percentage_for_stall(
    const float speed_deviation_in_percentage_for_stall)
  {
    _speed_deviation_in_percentage_for_stall = speed_deviation_in_percentage_for_stall;
  }

  uint32_t MotorMonitorConfig::get_active_speed_threshold() const
  {
    return _active_speed_threshold;
  }

  uint32_t MotorMonitorConfig::get_lower_position_threshold() const
  {
    return _lower_position_threshold;
  }

  uint32_t MotorMonitorConfig::get_max_time_diff_between_encoder_measurements_in_ms() const
  {
    return _max_time_diff_between_encoder_measurements_in_ms;
  }

  float MotorMonitorConfig::get_speed_deviation_in_percentage_for_stall() const
  {
    return _speed_deviation_in_percentage_for_stall;
  }

}   // namespace drawer_controller