#ifndef DRAWER_CONTROLLER_ENCODER_HPP
#define DRAWER_CONTROLLER_ENCODER_HPP

#include <Arduino.h>
#include <ESP32Encoder.h>

#include <memory>

#define OPEN_LOOP_COUNT_DRAWER_MAX_EXTENT       85000
#define ENCODER_COUNT_DRAWER_MAX_EXTENT         86000
#define DRAWER_POSITION_OPEN_LOOP_INTEGRAL_GAIN 1000

namespace drawer_controller
{
  class Encoder
  {
   public:
    Encoder(const bool use_encoder, const uint8_t encoder_pin_a, uint8_t encoder_pin_b);

    void update_position(uint32_t active_speed);

    int32_t get_current_position();

    uint8_t get_normed_current_position();

    uint32_t convert_uint8_position_to_drawer_position_scale(uint8_t position) const;

    void set_current_position(int32_t position);

    void init_encoder_before_next_movement(bool is_drawer_moving_out);

    uint32_t get_count_drawer_max_extent() const;

   private:
    const bool _use_encoder;
    std::unique_ptr<ESP32Encoder> _esp32_encoder;

    int32_t _current_position_int32 = 0;

    bool _is_drawer_moving_out;

    uint32_t _last_timestamp;

    int32_t get_integrated_drawer_position(uint32_t active_speed);

    void update_position_from_encoder();
  };
}   // namespace drawer_controller

#endif   // DRAWER_CONTROLLER_ENCODER_HPP
