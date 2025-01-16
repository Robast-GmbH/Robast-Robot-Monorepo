#ifndef DRAWER_SPEED_CONTROLLER_HPP
#define DRAWER_SPEED_CONTROLLER_HPP

#include <memory>

#include "drawer/electrical_drawer_config.hpp"
#include "motor/encoder.hpp"
#include "motor/motor.hpp"

namespace drawer
{
  constexpr uint8_t MAX_SPEED_UINT8 = 255;
  constexpr uint8_t STALL_GUARD_DISABLED = 0;
  constexpr uint8_t TARGET_SPEED_ZERO = 0;

  class SpeedController
  {
   public:
    SpeedController(const std::shared_ptr<motor::Encoder> encoder,
                    const std::shared_ptr<stepper_motor::Motor> motor,
                    const std::shared_ptr<ElectricalDrawerConfig> e_drawer_config,
                    const std::shared_ptr<switch_lib::Switch> endstop_switch);

    void set_target_speed_and_direction(const uint8_t target_speed, const bool use_acceleration_ramp);

    void set_target_speed_with_decelerating_ramp(const uint8_t target_speed);

    void start_homing_movement(const uint8_t target_speed);

    bool handle_initial_drawer_homing();

    bool was_drawer_homed_once() const;

    void set_is_drawer_moving_out(const bool is_drawer_moving_out);

    bool is_drawer_moving_out() const;

   private:
    const std::shared_ptr<motor::Encoder> _encoder;
    const std::shared_ptr<stepper_motor::Motor> _motor;
    const std::shared_ptr<ElectricalDrawerConfig> _e_drawer_config;
    const std::shared_ptr<switch_lib::Switch> _endstop_switch;

    bool _drawer_was_homed_once = false;
    bool _is_drawer_moving_out;

    uint32_t get_normed_target_speed_uint32(const uint8_t target_speed) const;
  };

}   // namespace drawer

#endif   // DRAWER_SPEED_CONTROLLER_HPP