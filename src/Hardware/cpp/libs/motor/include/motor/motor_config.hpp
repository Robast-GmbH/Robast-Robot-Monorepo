#ifndef MOTOR_MOTOR_CONFIG_HPP
#define MOTOR_MOTOR_CONFIG_HPP

#include <cstdint>

namespace motor
{
  class MotorConfig
  {
   public:
    MotorConfig() = default;

    void set_is_shaft_direction_inverted(const bool is_shaft_direction_inverted);

    bool get_is_shaft_direction_inverted() const;

   private:
    bool _is_shaft_direction_inverted;
  };
}   // namespace motor

#endif   // MOTOR_MOTOR_CONFIG_HPP