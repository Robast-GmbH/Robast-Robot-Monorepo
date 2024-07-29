#ifndef DRAWER_CONTROLLER_MOTOR_MONITOR_CONFIGS_HPP
#define DRAWER_CONTROLLER_MOTOR_MONITOR_CONFIGS_HPP

#include <cstdint>

namespace drawer_controller
{
  class MotorMonitorConfigs
  {
   public:
    MotorMonitorConfigs() = default;

    void set_active_speed_threshold(const uint32_t active_speed_threshold);
    void set_max_time_diff_between_encoder_measurements_in_ms(const uint32_t max_time_diff_between_encoder_measurements_in_ms);
    void set_speed_deviation_in_percentage_for_stall(const float speed_deviation_in_percentage_for_stall);

    uint32_t get_active_speed_threshold() const;
    uint32_t get_max_time_diff_between_encoder_measurements_in_ms() const;
    float get_speed_deviation_in_percentage_for_stall() const;

   private:
    uint32_t _active_speed_threshold;
    uint32_t _max_time_diff_between_encoder_measurements_in_ms;
    float _speed_deviation_in_percentage_for_stall;
  };

}   // namespace drawer_controller

#endif   // DRAWER_CONTROLLER_MOTOR_MONITOR_CONFIGS_HPP
