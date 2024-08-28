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
    void set_tcoolthrs(const uint32_t tcoolthrs);
    void set_tpwmthrs(const uint32_t tpwmthrs);
    void set_microsteps(const uint16_t microsteps);
    void set_semin(const uint8_t semin);
    void set_semax(const uint8_t semax);
    void set_sedn(const uint8_t sedn);

    bool get_is_shaft_direction_inverted() const;
    uint32_t get_tcoolthrs() const;
    uint32_t get_tpwmthrs() const;
    uint16_t get_microsteps() const;
    uint8_t get_semin() const;
    uint8_t get_semax() const;
    uint8_t get_sedn() const;

   private:
    bool _is_shaft_direction_inverted;
    uint32_t _tcoolthrs;
    uint32_t _tpwmthrs;
    uint16_t _microsteps;
    uint8_t _semin;
    uint8_t _semax;
    uint8_t _sedn;
  };
}   // namespace motor

#endif   // MOTOR_MOTOR_CONFIG_HPP