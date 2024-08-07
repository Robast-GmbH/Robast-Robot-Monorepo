#ifndef MOTOR_MOTOR_MONITOR_HPP
#define MOTOR_MOTOR_MONITOR_HPP

#include <memory>

#include "debug/debug.hpp"
#include "motor/encoder.hpp"
#include "motor/enconder_config.hpp"
#include "motor/motor.hpp"
#include "motor/motor_monitor_config.hpp"

namespace motor
{
  class MotorMonitor
  {
   public:
    MotorMonitor(const std::shared_ptr<Encoder> encoder,
                 const std::shared_ptr<EncoderConfig> encoder_config,
                 const std::shared_ptr<stepper_motor::Motor> motor,
                 const std::shared_ptr<MotorMonitorConfig> motor_monitor_config);

    bool is_motor_stalled();

   private:
    const std::shared_ptr<Encoder> _encoder;
    const std::shared_ptr<EncoderConfig> _encoder_config;
    const std::shared_ptr<stepper_motor::Motor> _motor;
    const std::shared_ptr<MotorMonitorConfig> _motor_monitor_config;

    constexpr static float _FACTOR_BETWEEN_SPEEDS = 0.9;

    uint32_t _last_timestamp_ms = 0;
    int32_t _last_position = 0;

    bool are_monitor_conditions_met(uint32_t time_difference_in_ms, int32_t current_position);
  };

}   // namespace motor

#endif   // MOTOR_MOTOR_MONITOR_HPP
