#include "drawer/speed_controller.hpp"

namespace drawer
{
  SpeedController::SpeedController(const std::shared_ptr<motor::Encoder> encoder,
                                   const std::shared_ptr<stepper_motor::Motor> motor,
                                   const std::shared_ptr<ElectricalDrawerConfig> e_drawer_config,
                                   const std::shared_ptr<switch_lib::Switch> endstop_switch)
      : _encoder{encoder}, _motor{motor}, _e_drawer_config{e_drawer_config}, _endstop_switch{endstop_switch}
  {
  }

  void SpeedController::set_target_speed_and_direction(const uint8_t target_speed, const bool use_acceleration_ramp)
  {
    uint32_t normed_target_speed_uint32 = get_normed_target_speed_uint32(target_speed);
    _is_drawer_moving_out ? _motor->set_direction(stepper_motor::Direction::counter_clockwise)
                          : _motor->set_direction(stepper_motor::Direction::clockwise);

    if (use_acceleration_ramp)
    {
      _motor->set_target_speed_with_accelerating_ramp(normed_target_speed_uint32,
                                                      _e_drawer_config->get_drawer_default_acceleration());
    }
    else
    {
      _motor->set_target_speed_instantly(normed_target_speed_uint32);
    }

    _encoder->init_encoder_before_next_movement(_is_drawer_moving_out);
  }

  void SpeedController::set_target_speed_with_decelerating_ramp(const uint8_t target_speed)
  {
    _motor->set_target_speed_with_decelerating_ramp(get_normed_target_speed_uint32(target_speed),
                                                    _encoder->convert_uint8_position_to_drawer_position_scale(
                                                      _e_drawer_config->get_drawer_moving_in_deceleration_distance()),
                                                    _encoder->get_current_position());
  }

  void SpeedController::start_homing_movement(const uint8_t target_speed)
  {
    _motor->set_direction(stepper_motor::Direction::clockwise);
    _motor->set_target_speed_instantly(get_normed_target_speed_uint32(target_speed));
  }

  bool SpeedController::handle_initial_drawer_homing()
  {
    if (!_drawer_was_homed_once && _endstop_switch->is_switch_pressed())
    {
      _motor->set_target_speed_instantly(TARGET_SPEED_ZERO);
      _encoder->set_current_position(STALL_GUARD_DISABLED);
      _drawer_was_homed_once = true;
      debug_printf_green("[ElectricalDrawer]: Drawer was homed successfully!\n");
      return true;
    }
    return false;
  }

  bool SpeedController::was_drawer_homed_once() const
  {
    return _drawer_was_homed_once;
  }

  void SpeedController::set_is_drawer_moving_out(const bool is_drawer_moving_out)
  {
    _is_drawer_moving_out = is_drawer_moving_out;
  }

  bool SpeedController::is_drawer_moving_out() const
  {
    return _is_drawer_moving_out;
  }

  uint32_t SpeedController::get_normed_target_speed_uint32(const uint8_t target_speed) const
  {
    uint32_t max_speed = _e_drawer_config->get_drawer_max_speed();
    uint32_t target_speed_casted = static_cast<uint32_t>(target_speed);

    return (target_speed_casted * max_speed) / MAX_SPEED_UINT8;
  }
}   // namespace drawer
