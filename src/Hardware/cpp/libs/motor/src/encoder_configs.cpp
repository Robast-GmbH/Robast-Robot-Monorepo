#include "motor/enconder_configs.hpp"

namespace drawer_controller
{
  void EncoderConfigs::set_open_loop_count_drawer_max_extent(const uint32_t open_loop_count_drawer_max_extent)
  {
    _open_loop_count_drawer_max_extent = open_loop_count_drawer_max_extent;
  }

  void EncoderConfigs::set_encoder_count_drawer_max_extent(const uint32_t encoder_count_drawer_max_extent)
  {
    _encoder_count_drawer_max_extent = encoder_count_drawer_max_extent;
  }

  void EncoderConfigs::set_drawer_position_open_loop_integral_gain(
    const uint32_t drawer_position_open_loop_integral_gain)
  {
    _drawer_position_open_loop_integral_gain = drawer_position_open_loop_integral_gain;
  }

  void EncoderConfigs::set_drawer_push_in_encoder_check_interval_ms(
    const uint32_t drawer_push_in_encoder_check_interval_ms)
  {
    _drawer_push_in_encoder_check_interval_ms = drawer_push_in_encoder_check_interval_ms;
  }

  void EncoderConfigs::set_drawer_push_in_threshold_in_percent_of_max_extent(
    const float drawer_push_in_threshold_in_percent_of_max_extent)
  {
    _drawer_push_in_threshold_in_percent_of_max_extent = drawer_push_in_threshold_in_percent_of_max_extent;
  }

  uint32_t EncoderConfigs::get_open_loop_count_drawer_max_extent() const
  {
    return _open_loop_count_drawer_max_extent;
  }

  uint32_t EncoderConfigs::get_encoder_count_drawer_max_extent() const
  {
    return _encoder_count_drawer_max_extent;
  }

  uint32_t EncoderConfigs::get_drawer_position_open_loop_integral_gain() const
  {
    return _drawer_position_open_loop_integral_gain;
  }

  uint32_t EncoderConfigs::get_drawer_push_in_encoder_check_interval_ms() const
  {
    return _drawer_push_in_encoder_check_interval_ms;
  }

  float EncoderConfigs::get_drawer_push_in_threshold_in_percent_of_max_extent() const
  {
    return _drawer_push_in_threshold_in_percent_of_max_extent;
  }

}   // namespace drawer_controller
