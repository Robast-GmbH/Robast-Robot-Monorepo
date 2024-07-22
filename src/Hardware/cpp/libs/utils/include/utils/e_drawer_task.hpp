#ifndef DRAWER_CONTROLLER_E_DRAWER_TASK_HPP
#define DRAWER_CONTROLLER_E_DRAWER_TASK_HPP

namespace drawer_controller
{
  struct EDrawerTask
  {
    uint8_t target_position;
    uint8_t target_speed;
    uint8_t stall_guard_value;
    bool is_homing;

    // Default Constructor
    EDrawerTask() : target_position(0), target_speed(0), stall_guard_value(0), is_homing(false)
    {
    }

    // Parameterized Constructor
    EDrawerTask(const uint8_t target_position_input,
                const uint8_t target_speed_input,
                const uint8_t stall_guard_value_input)
        : target_position(target_position_input),
          target_speed(target_speed_input),
          stall_guard_value(stall_guard_value_input),
          is_homing(false)
    {
    }

    // Parameterized Constructor
    EDrawerTask(const uint8_t target_position_input,
                const uint8_t target_speed_input,
                const uint8_t stall_guard_value_input,
                const bool is_homing_input)
        : target_position(target_position_input),
          target_speed(target_speed_input),
          stall_guard_value(stall_guard_value_input),
          is_homing(is_homing_input)
    {
    }
  };
}   // namespace drawer_controller

#endif   // DRAWER_CONTROLLER_E_DRAWER_TASK_HPP
