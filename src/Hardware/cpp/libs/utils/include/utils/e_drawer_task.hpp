#ifndef DRAWER_CONTROLLER_E_DRAWER_TASK_HPP
#define DRAWER_CONTROLLER_E_DRAWER_TASK_HPP

namespace drawer_controller
{
    struct EDrawerTask
    {
        uint8_t target_position;
        uint8_t target_speed;
        bool stall_guard_enabled;

        // Default Constructor
        EDrawerTask() : target_position(0), target_speed(0), stall_guard_enabled(false)
        {
        }

        // Parameterized Constructor
        EDrawerTask(const uint8_t target_position_input,
                    const uint8_t target_speed_input,
                    const uint8_t stall_guard_enabled_input)
            : target_position(target_position_input),
              target_speed(target_speed_input)
        {
            stall_guard_enabled = stall_guard_enabled_input == CAN_DATA_ELECTRICAL_DRAWER_STALL_GUARD_ENABLED;
        }
    };
} // namespace drawer_controller

#endif // DRAWER_CONTROLLER_E_DRAWER_TASK_HPP
