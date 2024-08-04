#ifndef DRAWER_CONTROLLER_MOTOR_CONFIG_HPP
#define DRAWER_CONTROLLER_MOTOR_CONFIG_HPP

#include <cstdint>

namespace drawer_controller
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
}   // namespace drawer_controller

#endif   // DRAWER_CONTROLLER_MOTOR_CONFIG_HPP