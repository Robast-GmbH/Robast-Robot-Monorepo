#include "motor/motor_monitor.hpp"

namespace motor
{
  MotorMonitor::MotorMonitor(const std::shared_ptr<Encoder> encoder,
                             const std::shared_ptr<EncoderConfig> encoder_config,
                             const std::shared_ptr<stepper_motor::Motor> motor,
                             const std::shared_ptr<MotorMonitorConfig> motor_monitor_config)
      : _encoder(encoder), _encoder_config(encoder_config), _motor(motor), _motor_monitor_config(motor_monitor_config)
  {
  }

  bool MotorMonitor::is_motor_stalled()
  {
    const int32_t current_position = _encoder->get_current_position();

    const uint32_t current_timestamp_ms = millis();

    const uint32_t active_motor_speed = _motor->get_active_speed();
    const float active_motor_speed_float = static_cast<float>(active_motor_speed);

    const uint32_t position_difference = abs(current_position - _last_position);
    const uint32_t time_difference_in_ms = current_timestamp_ms - _last_timestamp_ms;

    if (are_monitor_conditions_met(time_difference_in_ms, current_position))
    {
      if (position_difference == 0)
      {
        debug_printf_color(ANSI_COLOR_MAGENTA,
                           "[MotorMonitor]: Stall guard is triggered because the position difference is 0!");
        return true;
      }

      const uint32_t encoder_speed = (position_difference * 1000) / time_difference_in_ms;

      const uint32_t active_speed_threshold = _motor_monitor_config->get_active_speed_threshold();

      if (encoder_speed > active_speed_threshold && active_motor_speed > active_speed_threshold)
      {
        const float active_motor_speed_normed_to_encoder_speed = active_motor_speed_float * _FACTOR_BETWEEN_SPEEDS;

        const float speed_deviation = _motor_monitor_config->get_speed_deviation_in_percentage_for_stall();

        const uint32_t lower_speed_limit =
          static_cast<uint32_t>(active_motor_speed_normed_to_encoder_speed * (1.0f - speed_deviation));

        if (encoder_speed < lower_speed_limit)
        {
          debug_printf_color(
            ANSI_COLOR_MAGENTA,
            "[MotorMonitor]: Motor is stalled! Target motor speed: %u. Active motor speed: %u, Active Motor Speed "
            "normed to encoder speed: %f, Encoder Speed: %u. Deviation is higher than %f percent. Time difference "
            "between encoder measurements in ms: %d. Max time diff: %d. Position difference: "
            "%d. Current position: %d. Last Position: %d. lower_speed_limit: %u\n",
            _motor->get_target_speed(),
            active_motor_speed,
            active_motor_speed_normed_to_encoder_speed,
            encoder_speed,
            speed_deviation * 100,
            time_difference_in_ms,
            _motor_monitor_config->get_max_time_diff_between_encoder_measurements_in_ms(),
            position_difference,
            current_position,
            _last_position,
            lower_speed_limit);
          return true;
        }
      }
    }
    _last_position = current_position;
    _last_timestamp_ms = current_timestamp_ms;

    return false;
  }

  bool MotorMonitor::are_monitor_conditions_met(uint32_t time_difference_in_ms, int32_t current_position)
  {
    uint32_t max_time_diff = _motor_monitor_config->get_max_time_diff_between_encoder_measurements_in_ms();

    uint32_t lower_position_threshold = _motor_monitor_config->get_lower_position_threshold();

    return (time_difference_in_ms > 0) && (time_difference_in_ms < max_time_diff) &&
           (current_position > lower_position_threshold);
  }
}   // namespace motor