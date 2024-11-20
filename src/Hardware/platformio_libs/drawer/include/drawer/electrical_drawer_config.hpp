#ifndef DRAWER_ELECTRICAL_DRAWER_CONFIG_HPP
#define DRAWER_ELECTRICAL_DRAWER_CONFIG_HPP

#include <cstdint>

namespace drawer
{
  class ElectricalDrawerConfig
  {
   public:
    ElectricalDrawerConfig() = default;

    void set_drawer_max_speed(const uint32_t max_speed);
    void set_drawer_homing_speed(const uint32_t homing_speed);
    void set_drawer_initial_homing_speed(const uint32_t initial_homing_speed);
    void set_drawer_moving_in_deceleration_distance(const uint8_t deceleration_distance);
    void set_drawer_moving_in_final_homing_distance(const uint8_t final_homing_distance);
    void set_drawer_moving_out_deceleration_distance(const uint8_t deceleration_distance);
    void set_drawer_moving_out_final_speed(const uint64_t final_speed);
    void set_drawer_push_in_auto_close_speed(const uint8_t auto_close_speed);
    void set_drawer_push_in_auto_close_stall_guard_value(const uint8_t stall_guard_value);
    void set_drawer_push_in_wait_time_after_stall_guard_triggered_in_ms(const uint32_t wait_time);
    void set_drawer_push_in_wait_time_after_movement_finished_in_ms(const uint32_t wait_time);
    void set_drawer_stall_guard_wait_time_after_movement_started_in_ms(const uint32_t wait_time);
    void set_use_tmc_stall_guard(const bool use_tmc_stall_guard);
    void set_use_motor_monitor_stall_guard(const bool use_motor_monitor_stall_guard);
    void set_encoder_threshold_for_drawer_not_opened_during_stall(const uint8_t threshold);
    void set_drawer_default_acceleration(const uint8_t acceleration);
    void set_wait_time_to_close_lock_after_drawer_opened_in_ms(const uint32_t wait_time);

    uint32_t get_drawer_max_speed() const;
    uint32_t get_drawer_homing_speed() const;
    uint32_t get_drawer_initial_homing_speed() const;
    uint8_t get_drawer_moving_in_deceleration_distance() const;
    uint8_t get_drawer_moving_in_final_homing_distance() const;
    uint8_t get_drawer_moving_out_deceleration_distance() const;
    uint64_t get_drawer_moving_out_final_speed() const;
    uint8_t get_drawer_push_in_auto_close_speed() const;
    uint8_t get_drawer_push_in_auto_close_stall_guard_value() const;
    uint32_t get_drawer_push_in_wait_time_after_stall_guard_triggered_in_ms() const;
    uint32_t get_drawer_push_in_wait_time_after_movement_finished_in_ms() const;
    uint32_t get_drawer_stall_guard_wait_time_after_movement_started_in_ms() const;
    bool get_use_tmc_stall_guard() const;
    bool get_use_motor_monitor_stall_guard() const;
    uint8_t get_encoder_threshold_for_drawer_not_opened_during_stall() const;
    uint8_t get_drawer_default_acceleration() const;
    uint32_t get_wait_time_to_close_lock_after_drawer_opened_in_ms() const;

   private:
    uint64_t _max_speed;
    uint64_t _homing_speed;
    uint64_t _initial_homing_speed;
    uint8_t _moving_in_deceleration_distance;
    uint8_t _moving_in_final_homing_distance;
    uint8_t _moving_out_deceleration_distance;
    uint64_t _moving_out_final_speed;
    uint8_t _push_in_auto_close_speed;
    uint8_t _push_in_auto_close_stall_guard_value;
    uint32_t _push_in_wait_time_after_stall_guard_triggered_in_ms;
    uint32_t _push_in_wait_time_after_movement_finished_in_ms;
    uint32_t _stall_guard_wait_time_after_movement_started_in_ms;
    bool _use_tmc_stall_guard;
    bool _use_motor_monitor_stall_guard;
    uint8_t _encoder_threshold_for_drawer_not_opened_during_stall;
    uint8_t _drawer_default_acceleration;
    uint32_t _wait_time_to_close_lock_after_drawer_opened_in_ms;
  };
  ;

}   // namespace drawer

#endif   // DRAWER_ELECTRICAL_DRAWER_CONFIG_HPP