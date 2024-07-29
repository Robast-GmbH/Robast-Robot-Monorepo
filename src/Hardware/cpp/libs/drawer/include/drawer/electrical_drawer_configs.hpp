#ifndef DRAWER_CONTROLLER_ELECTRICAL_DRAWER_CONFIGS_HPP
#define DRAWER_CONTROLLER_ELECTRICAL_DRAWER_CONFIGS_HPP

#include <cstdint>

namespace drawer_controller
{
  class ElectricalDrawerConfigs
  {
   public:
    ElectricalDrawerConfigs() = default;

    void set_drawer_max_speed(const uint32_t max_speed);
    void set_drawer_homing_speed(const uint32_t homing_speed);
    void set_drawer_initial_homing_speed(const uint32_t initial_homing_speed);
    void set_drawer_moving_in_deceleration_distance(const uint8_t deceleration_distance);
    void set_drawer_moving_in_final_homing_distance(const uint8_t final_homing_distance);
    void set_drawer_moving_out_deceleration_distance(const uint8_t deceleration_distance);
    void set_drawer_push_in_auto_close_speed(const uint8_t auto_close_speed);
    void set_drawer_push_in_auto_close_stall_guard_value(const uint8_t stall_guard_value);
    void set_drawer_push_in_wait_time_after_stall_guard_triggered_in_ms(const uint32_t wait_time);

    uint32_t get_drawer_max_speed() const;
    uint32_t get_drawer_homing_speed() const;
    uint32_t get_drawer_initial_homing_speed() const;
    uint8_t get_drawer_moving_in_deceleration_distance() const;
    uint8_t get_drawer_moving_in_final_homing_distance() const;
    uint8_t get_drawer_moving_out_deceleration_distance() const;
    uint8_t get_drawer_push_in_auto_close_speed() const;
    uint8_t get_drawer_push_in_auto_close_stall_guard_value() const;
    uint32_t get_drawer_push_in_wait_time_after_stall_guard_triggered_in_ms() const;

   private:
    uint64_t _max_speed;
    uint64_t _homing_speed;
    uint64_t _initial_homing_speed;
    uint8_t _moving_in_deceleration_distance;
    uint8_t _moving_in_final_homing_distance;
    uint8_t _moving_out_deceleration_distance;
    uint8_t _push_in_auto_close_speed;
    uint8_t _push_in_auto_close_stall_guard_value;
    uint32_t _push_in_wait_time_after_stall_guard_triggered_in_ms;
  };

}   // namespace drawer_controller

#endif   // DRAWER_CONTROLLER_ELECTRICAL_DRAWER_CONFIGS_HPP