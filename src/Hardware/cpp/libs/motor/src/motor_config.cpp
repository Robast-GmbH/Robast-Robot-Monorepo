#include "motor/motor_config.hpp"

namespace motor
{
  void MotorConfig::set_is_shaft_direction_inverted(const bool is_shaft_direction_inverted)
  {
    _is_shaft_direction_inverted = is_shaft_direction_inverted;
  }

  bool MotorConfig::get_is_shaft_direction_inverted() const
  {
    return _is_shaft_direction_inverted;
  }

}   // namespace motor