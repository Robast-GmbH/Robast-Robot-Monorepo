#ifndef MOTOR_MOTOR_MONITOR_CONFIG_HPP
#define MOTOR_MOTOR_MONITOR_CONFIG_HPP

#include <cstdint>

namespace motor
{
  class MotorMonitorConfig
  {
   public:
    MotorMonitorConfig() = default;

    void set_active_speed_threshold(const uint32_t active_speed_threshold);
    void set_lower_position_threshold(const uint32_t lower_position_threshold);
    void set_max_time_diff_between_encoder_measurements_in_ms(
      const uint32_t max_time_diff_between_encoder_measurements_in_ms);
    void set_speed_deviation_in_percentage_for_stall(const float speed_deviation_in_percentage_for_stall);

    uint32_t get_active_speed_threshold() const;
    uint32_t get_lower_position_threshold() const;
    uint32_t get_max_time_diff_between_encoder_measurements_in_ms() const;
    float get_speed_deviation_in_percentage_for_stall() const;

   private:
    uint32_t _active_speed_threshold;
    uint32_t _lower_position_threshold;
    uint32_t _max_time_diff_between_encoder_measurements_in_ms;
    float _speed_deviation_in_percentage_for_stall;
  };

}   // namespace motor

#endif   // MOTOR_MOTOR_MONITOR_CONFIG_HPP
