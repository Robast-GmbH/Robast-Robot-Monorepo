#include "motor/motor_configs.hpp"

namespace drawer_controller
{
  void MotorConfigs::set_is_shaft_direction_inverted(const bool is_shaft_direction_inverted)
  {
    _is_shaft_direction_inverted = is_shaft_direction_inverted;
  }

  bool MotorConfigs::get_is_shaft_direction_inverted() const
  {
    return _is_shaft_direction_inverted;
  }

}   // namespace drawer_controller