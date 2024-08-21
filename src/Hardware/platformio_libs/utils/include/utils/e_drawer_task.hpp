#ifndef UTILS_E_DRAWER_TASK_HPP
#define UTILS_E_DRAWER_TASK_HPP

namespace utils
{
  struct EDrawerTask
  {
    uint8_t target_position;
    uint8_t target_speed;
    uint8_t stall_guard_value;
    bool is_homing;
    bool use_acceleration_ramp;

    // Default Constructor
    EDrawerTask()
        : target_position(0), target_speed(0), stall_guard_value(0), is_homing(false), use_acceleration_ramp(true)
    {
    }

    // Parameterized Constructor
    EDrawerTask(const uint8_t target_position_arg,
                const uint8_t target_speed_arg,
                const uint8_t stall_guard_value_arg)
        : target_position(target_position_arg),
          target_speed(target_speed_arg),
          stall_guard_value(stall_guard_value_arg),
          is_homing(false),
          use_acceleration_ramp(true)
    {
    }

    // Parameterized Constructor
    EDrawerTask(const uint8_t target_position_arg,
                const uint8_t target_speed_arg,
                const uint8_t stall_guard_value_arg,
                const bool is_homing_input,
                const bool use_acceleration_ramp_input)
        : target_position(target_position_arg),
          target_speed(target_speed_arg),
          stall_guard_value(stall_guard_value_arg),
          is_homing(is_homing_input),
          use_acceleration_ramp(use_acceleration_ramp_input)
    {
    }
  };
}   // namespace utils

#endif   // UTILS_E_DRAWER_TASK_HPP
