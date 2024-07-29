#include "motor/motor_monitor.hpp"

namespace drawer_controller
{
  MotorMonitor::MotorMonitor(const std::shared_ptr<Encoder> encoder,
                             const std::shared_ptr<EncoderConfigs> encoder_configs,
                             const std::shared_ptr<stepper_motor::Motor> motor,
                             const std::shared_ptr<MotorMonitorConfigs> motor_monitor_configs)
      : _encoder(encoder),
        _encoder_configs(encoder_configs),
        _motor(motor),
        _motor_monitor_configs(motor_monitor_configs)
  {
  }

  bool MotorMonitor::is_motor_stalled()
  {
    int32_t current_position = _encoder->get_current_position();

    uint32_t current_timestamp_ms = millis();

    uint32_t active_motor_speed = _motor->get_active_speed();

    uint32_t position_difference = abs(current_position - _last_position);
    uint32_t time_difference_in_ms = current_timestamp_ms - _last_timestamp_ms;

    uint32_t max_time_diff = _motor_monitor_configs->get_max_time_diff_between_encoder_measurements_in_ms();

    if ((time_difference_in_ms > 0) && (time_difference_in_ms < max_time_diff))
    {
      uint32_t encoder_speed = (position_difference * 1000) / time_difference_in_ms;

      if (encoder_speed > _motor_monitor_configs->get_active_speed_threshold())
      {
        uint32_t active_motor_speed_normed_to_encoder_speed =
          static_cast<uint32_t>(std::round(active_motor_speed * _FACTOR_BETWEEN_SPEEDS));

        float speed_deviation = _motor_monitor_configs->get_speed_deviation_in_percentage_for_stall();

        uint32_t upper_speed_limit = static_cast<uint32_t>(std::round(encoder_speed * (1 + speed_deviation)));
        uint32_t lower_speed_limit = static_cast<uint32_t>(std::round(encoder_speed * (1 - speed_deviation)));

        if ((active_motor_speed_normed_to_encoder_speed > upper_speed_limit ||
             (active_motor_speed_normed_to_encoder_speed < lower_speed_limit)))
        {
          debug_printf(
            "[MotorMonitor]: Motor is stalled! Active Motor Speed: %d, Encoder Speed: %d. Deviation is higher than "
            "%.1f%%. Time difference between encoder measurements in ms: %d. Max time diff: %d. Position difference: "
            "%d.\n",
            active_motor_speed_normed_to_encoder_speed,
            encoder_speed,
            speed_deviation * 100,
            time_difference_in_ms,
            max_time_diff,
            position_difference);
          return true;
        }
      }
    }
    _last_position = current_position;
    _last_timestamp_ms = current_timestamp_ms;

    return false;
  }
}   // namespace drawer_controller