#ifndef MOTOR_ENCODER_CONFIG_HPP
#define MOTOR_ENCODER_CONFIG_HPP

#include <cstdint>

#include "debug/debug.hpp"

namespace motor
{
  class EncoderConfig
  {
   public:
    EncoderConfig() = default;

    void print_all_configs() const;

    void set_open_loop_count_drawer_max_extent(const uint32_t open_loop_count_drawer_max_extent);
    void set_encoder_count_drawer_max_extent(const uint32_t encoder_count_drawer_max_extent);
    void set_drawer_position_open_loop_integral_gain(const uint32_t drawer_position_open_loop_integral_gain);
    void set_drawer_push_in_threshold_in_percent_of_max_extent(
      const float drawer_push_in_threshold_in_percent_of_max_extent);
    void set_drawer_push_in_encoder_check_interval_ms(const uint32_t drawer_push_in_encoder_check_interval_ms);
    void set_drawer_pulled_out_threshold_in_percent_of_max_extent(
      const float drawer_pulled_out_threshold_in_percent_of_max_extent);

    uint32_t get_open_loop_count_drawer_max_extent() const;
    uint32_t get_encoder_count_drawer_max_extent() const;
    uint32_t get_drawer_position_open_loop_integral_gain() const;
    float get_drawer_push_in_threshold_in_percent_of_max_extent() const;
    uint32_t get_drawer_push_in_encoder_check_interval_ms() const;
    float get_drawer_pulled_out_threshold_in_percent_of_max_extent() const;

   private:
    uint32_t _open_loop_count_drawer_max_extent;
    uint32_t _encoder_count_drawer_max_extent;

    uint32_t _drawer_position_open_loop_integral_gain;

    float _drawer_push_in_threshold_in_percent_of_max_extent;
    uint32_t _drawer_push_in_encoder_check_interval_ms;

    float _drawer_pulled_out_threshold_in_percent_of_max_extent;
  };

}   // namespace motor

#endif   // MOTOR_ENCODER_CONFIG_HPP