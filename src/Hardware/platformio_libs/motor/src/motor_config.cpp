#include "motor/motor_config.hpp"

namespace motor
{
  void MotorConfig::set_is_shaft_direction_inverted(const bool is_shaft_direction_inverted)
  {
    _is_shaft_direction_inverted = is_shaft_direction_inverted;
  }

  void MotorConfig::set_tcoolthrs(const uint32_t tcoolthrs)
  {
    _tcoolthrs = tcoolthrs;
  }

  void MotorConfig::set_tpwmthrs(const uint32_t tpwmthrs)
  {
    _tpwmthrs = tpwmthrs;
  }

  void MotorConfig::set_microsteps(const uint16_t microsteps)
  {
    _microsteps = microsteps;
  }

  void MotorConfig::set_semin(const uint8_t semin)
  {
    _semin = semin;
  }

  void MotorConfig::set_semax(const uint8_t semax)
  {
    _semax = semax;
  }

  void MotorConfig::set_sedn(const uint8_t sedn)
  {
    _sedn = sedn;
  }

  bool MotorConfig::get_is_shaft_direction_inverted() const
  {
    return _is_shaft_direction_inverted;
  }

  uint32_t MotorConfig::get_tcoolthrs() const
  {
    return _tcoolthrs;
  }

  uint32_t MotorConfig::get_tpwmthrs() const
  {
    return _tpwmthrs;
  }

  uint16_t MotorConfig::get_microsteps() const
  {
    return _microsteps;
  }

  uint8_t MotorConfig::get_semin() const
  {
    return _semin;
  }

  uint8_t MotorConfig::get_semax() const
  {
    return _semax;
  }

  uint8_t MotorConfig::get_sedn() const
  {
    return _sedn;
  }

}   // namespace motor