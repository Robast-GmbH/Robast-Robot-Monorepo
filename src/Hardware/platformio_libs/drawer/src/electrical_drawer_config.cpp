#include "drawer/electrical_drawer_config.hpp"

namespace drawer
{
  void ElectricalDrawerConfig::set_drawer_max_speed(const uint32_t max_speed)
  {
    _max_speed = max_speed;
  }

  void ElectricalDrawerConfig::set_drawer_homing_speed(const uint32_t homing_speed)
  {
    _homing_speed = homing_speed;
  }

  void ElectricalDrawerConfig::set_drawer_initial_homing_speed(const uint32_t initial_homing_speed)
  {
    _initial_homing_speed = initial_homing_speed;
  }

  void ElectricalDrawerConfig::set_drawer_moving_in_deceleration_distance(const uint8_t deceleration_distance)
  {
    _moving_in_deceleration_distance = deceleration_distance;
  }

  void ElectricalDrawerConfig::set_drawer_moving_in_final_homing_distance(const uint8_t final_homing_distance)
  {
    _moving_in_final_homing_distance = final_homing_distance;
  }

  void ElectricalDrawerConfig::set_drawer_moving_out_deceleration_distance(const uint8_t deceleration_distance)
  {
    _moving_out_deceleration_distance = deceleration_distance;
  }

  void ElectricalDrawerConfig::set_drawer_moving_out_final_speed(const uint64_t final_speed)
  {
    _moving_out_final_speed = final_speed;
  }

  void ElectricalDrawerConfig::set_drawer_push_in_auto_close_speed(const uint8_t auto_close_speed)
  {
    _push_in_auto_close_speed = auto_close_speed;
  }

  void ElectricalDrawerConfig::set_drawer_push_in_auto_close_stall_guard_value(const uint8_t stall_guard_value)
  {
    _push_in_auto_close_stall_guard_value = stall_guard_value;
  }

  void ElectricalDrawerConfig::set_drawer_push_in_wait_time_after_stall_guard_triggered_in_ms(const uint32_t wait_time)
  {
    _push_in_wait_time_after_stall_guard_triggered_in_ms = wait_time;
  }

  void ElectricalDrawerConfig::set_drawer_push_in_wait_time_after_movement_finished_in_ms(const uint32_t wait_time)
  {
    _push_in_wait_time_after_movement_finished_in_ms = wait_time;
  }

  void ElectricalDrawerConfig::set_drawer_stall_guard_wait_time_after_movement_started_in_ms(const uint32_t wait_time)
  {
    _stall_guard_wait_time_after_movement_started_in_ms = wait_time;
  }

  void ElectricalDrawerConfig::set_use_tmc_stall_guard(const bool use_tmc_stall_guard)
  {
    _use_tmc_stall_guard = use_tmc_stall_guard;
  }

  void ElectricalDrawerConfig::set_use_motor_monitor_stall_guard(const bool use_motor_monitor_stall_guard)
  {
    _use_motor_monitor_stall_guard = use_motor_monitor_stall_guard;
  }

  void ElectricalDrawerConfig::set_encoder_threshold_for_drawer_not_opened_during_stall(const uint8_t threshold)
  {
    _encoder_threshold_for_drawer_not_opened_during_stall = threshold;
  }

  void ElectricalDrawerConfig::set_drawer_default_acceleration(const uint8_t acceleration)
  {
    _drawer_default_acceleration = acceleration;
  }

  void ElectricalDrawerConfig::set_wait_time_to_close_lock_after_drawer_opened_in_ms(const uint32_t wait_time)
  {
    _wait_time_to_close_lock_after_drawer_opened_in_ms = wait_time;
  }

  uint32_t ElectricalDrawerConfig::get_drawer_max_speed() const
  {
    return _max_speed;
  }

  uint32_t ElectricalDrawerConfig::get_drawer_homing_speed() const
  {
    return _homing_speed;
  }

  uint32_t ElectricalDrawerConfig::get_drawer_initial_homing_speed() const
  {
    return _initial_homing_speed;
  }

  uint8_t ElectricalDrawerConfig::get_drawer_moving_in_deceleration_distance() const
  {
    return _moving_in_deceleration_distance;
  }

  uint8_t ElectricalDrawerConfig::get_drawer_moving_in_final_homing_distance() const
  {
    return _moving_in_final_homing_distance;
  }

  uint8_t ElectricalDrawerConfig::get_drawer_moving_out_deceleration_distance() const
  {
    return _moving_out_deceleration_distance;
  }

  uint64_t ElectricalDrawerConfig::get_drawer_moving_out_final_speed() const
  {
    return _moving_out_final_speed;
  }

  uint8_t ElectricalDrawerConfig::get_drawer_push_in_auto_close_speed() const
  {
    return _push_in_auto_close_speed;
  }

  uint8_t ElectricalDrawerConfig::get_drawer_push_in_auto_close_stall_guard_value() const
  {
    return _push_in_auto_close_stall_guard_value;
  }

  uint32_t ElectricalDrawerConfig::get_drawer_push_in_wait_time_after_stall_guard_triggered_in_ms() const
  {
    return _push_in_wait_time_after_stall_guard_triggered_in_ms;
  }

  uint32_t ElectricalDrawerConfig::get_drawer_push_in_wait_time_after_movement_finished_in_ms() const
  {
    return _push_in_wait_time_after_movement_finished_in_ms;
  }

  uint32_t ElectricalDrawerConfig::get_drawer_stall_guard_wait_time_after_movement_started_in_ms() const
  {
    return _stall_guard_wait_time_after_movement_started_in_ms;
  }

  bool ElectricalDrawerConfig::get_use_tmc_stall_guard() const
  {
    return _use_tmc_stall_guard;
  }

  bool ElectricalDrawerConfig::get_use_motor_monitor_stall_guard() const
  {
    return _use_motor_monitor_stall_guard;
  }

  uint8_t ElectricalDrawerConfig::get_encoder_threshold_for_drawer_not_opened_during_stall() const
  {
    return _encoder_threshold_for_drawer_not_opened_during_stall;
  }

  uint8_t ElectricalDrawerConfig::get_drawer_default_acceleration() const
  {
    return _drawer_default_acceleration;
  }

  uint32_t ElectricalDrawerConfig::get_wait_time_to_close_lock_after_drawer_opened_in_ms() const
  {
    return _wait_time_to_close_lock_after_drawer_opened_in_ms;
  }
}   // namespace drawer