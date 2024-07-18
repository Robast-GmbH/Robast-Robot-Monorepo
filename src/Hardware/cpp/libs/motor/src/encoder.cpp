#include "motor/encoder.hpp"

namespace drawer_controller
{
  Encoder::Encoder(const bool use_encoder, const uint8_t encoder_pin_a, const uint8_t encoder_pin_b)
      : _use_encoder(use_encoder)
  {
    if (use_encoder)
    {
      _esp32_encoder = std::make_unique<ESP32Encoder>(true);
      ESP32Encoder::useInternalWeakPullResistors = UP;
      _esp32_encoder->attachFullQuad(encoder_pin_a, encoder_pin_b);
      _esp32_encoder->setCount(0);
    }
  }

  void Encoder::update_position(uint32_t active_speed)
  {
    if (_use_encoder)
    {
      _current_position_int32 = abs(_esp32_encoder->getCount());
    }
    else
    {
      _current_position_int32 += get_integrated_drawer_position(active_speed);
      if (_current_position_int32 < 0)
      {
        _current_position_int32 = 0;
      }
    }
  }

  int32_t Encoder::get_current_position() const
  {
    return _current_position_int32;
  }

  uint8_t Encoder::get_normed_current_position() const
  {
    uint32_t scale = _use_encoder ? ENCODER_COUNT_DRAWER_MAX_EXTENT : OPEN_LOOP_COUNT_DRAWER_MAX_EXTENT;

    uint32_t normed_current_position_uint32 = (_current_position_int32 * UINT8_MAX) / scale;

    if (normed_current_position_uint32 > UINT8_MAX)
    {
      return UINT8_MAX;
    }
    else
    {
      return normed_current_position_uint32;
    }
  }

  uint32_t Encoder::convert_uint8_position_to_drawer_position_scale(uint8_t position) const
  {
    uint32_t scale = _use_encoder ? ENCODER_COUNT_DRAWER_MAX_EXTENT : OPEN_LOOP_COUNT_DRAWER_MAX_EXTENT;

    return (position * scale) / UINT8_MAX;
  }

  void Encoder::set_current_position(int32_t position)
  {
    if (_use_encoder)
    {
      _esp32_encoder->setCount(position);
    }
    _current_position_int32 = position;
  }

  void Encoder::init_encoder_before_next_movement(bool is_drawer_moving_out)
  {
    _is_drawer_moving_out = is_drawer_moving_out;
    if (!_use_encoder)
    {
      _last_timestemp = millis();
    }
  }

  int32_t Encoder::get_integrated_drawer_position(uint32_t active_speed)
  {
    int32_t integrated_position = 0;
    uint32_t current_timestemp = millis();

    integrated_position =
      ((current_timestemp - _last_timestemp) * active_speed) / DRAWER_POSITION_OPEN_LOOP_INTEGRAL_GAIN;

    if (!_is_drawer_moving_out)
    {
      integrated_position *= -1;
    }
    _last_timestemp = current_timestemp;

    return integrated_position;
  }

  uint32_t Encoder::get_count_drawer_max_extent() const
  {
    return _use_encoder ? ENCODER_COUNT_DRAWER_MAX_EXTENT : OPEN_LOOP_COUNT_DRAWER_MAX_EXTENT;
  }

}   // namespace drawer_controller