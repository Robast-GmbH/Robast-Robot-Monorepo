#include "drawer/electrical_drawer_configs.hpp"

namespace drawer_controller
{
    void ElectricalDrawerConfigs::set_drawer_max_speed(const uint32_t max_speed)
    {
        _max_speed = max_speed;
    }

    void ElectricalDrawerConfigs::set_drawer_homing_speed(const uint32_t homing_speed)
    {
        _homing_speed = homing_speed;
    }

    void ElectricalDrawerConfigs::set_drawer_initial_homing_speed(const uint32_t initial_homing_speed)
    {
        _initial_homing_speed = initial_homing_speed;
    }

    void ElectricalDrawerConfigs::set_drawer_moving_in_deceleration_distance(const uint8_t deceleration_distance)
    {
        _moving_in_deceleration_distance = deceleration_distance;
    }

    void ElectricalDrawerConfigs::set_drawer_moving_in_final_homing_distance(const uint8_t final_homing_distance)
    {
        _moving_in_final_homing_distance = final_homing_distance;
    }

    void ElectricalDrawerConfigs::set_drawer_moving_out_deceleration_distance(const uint8_t deceleration_distance)
    {
        _moving_out_deceleration_distance = deceleration_distance;
    }

    void ElectricalDrawerConfigs::set_drawer_push_in_auto_close_speed(const uint8_t auto_close_speed)
    {
        _push_in_auto_close_speed = auto_close_speed;
    }

    void ElectricalDrawerConfigs::set_drawer_push_in_auto_close_stall_guard_value(const uint8_t stall_guard_value)
    {
        _push_in_auto_close_stall_guard_value = stall_guard_value;
    }

    void ElectricalDrawerConfigs::set_drawer_push_in_threshold_in_percent_of_max_extent(const float threshold_in_percent_of_max_extent)
    {
        _push_in_threshold_in_percent_of_max_extent = threshold_in_percent_of_max_extent;
    }

    void ElectricalDrawerConfigs::set_drawer_push_in_encoder_check_interval_ms(const uint32_t encoder_check_interval_ms)
    {
        _push_in_encoder_check_interval_ms = encoder_check_interval_ms;
    }

    uint32_t ElectricalDrawerConfigs::get_drawer_max_speed() const
    {
        return _max_speed;
    }

    uint32_t ElectricalDrawerConfigs::get_drawer_homing_speed() const
    {
        return _homing_speed;
    }

    uint32_t ElectricalDrawerConfigs::get_drawer_initial_homing_speed() const
    {
        return _initial_homing_speed;
    }

    uint8_t ElectricalDrawerConfigs::get_drawer_moving_in_deceleration_distance() const
    {
        return _moving_in_deceleration_distance;
    }

    uint8_t ElectricalDrawerConfigs::get_drawer_moving_in_final_homing_distance() const
    {
        return _moving_in_final_homing_distance;
    }

    uint8_t ElectricalDrawerConfigs::get_drawer_moving_out_deceleration_distance() const
    {
        return _moving_out_deceleration_distance;
    }

    uint8_t ElectricalDrawerConfigs::get_drawer_push_in_auto_close_speed() const
    {
        return _push_in_auto_close_speed;
    }

    uint8_t ElectricalDrawerConfigs::get_drawer_push_in_auto_close_stall_guard_value() const
    {
        return _push_in_auto_close_stall_guard_value;
    }

    float ElectricalDrawerConfigs::get_drawer_push_in_threshold_in_percent_of_max_extent() const
    {
        return _push_in_threshold_in_percent_of_max_extent;
    }

    uint32_t ElectricalDrawerConfigs::get_drawer_push_in_encoder_check_interval_ms() const
    {
        return _push_in_encoder_check_interval_ms;
    }

}