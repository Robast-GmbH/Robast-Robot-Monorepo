#include "motor/encoder.hpp"

#include "debug/debug.hpp"

namespace motor
{
  Encoder::Encoder(const bool use_encoder,
                   const uint8_t encoder_pin_a,
                   const uint8_t encoder_pin_b,
                   const std::shared_ptr<EncoderConfig> config)
      : _use_encoder(use_encoder), _config(config)
  {
    if (use_encoder)
    {
      _esp32_encoder = std::make_unique<ESP32Encoder>(true);
      ESP32Encoder::isrServiceCpuCore = 0;   // Use core 0
      ESP32Encoder::useInternalWeakPullResistors = UP;
      _esp32_encoder->attachFullQuad(encoder_pin_a, encoder_pin_b);
      _esp32_encoder->setCount(0);
    }
  }

  void Encoder::update_position(uint32_t active_speed)
  {
    if (_use_encoder)
    {
      update_position_from_encoder();
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

  void Encoder::update_position_from_encoder()
  {
    _current_position_int32 = -_esp32_encoder->getCount();
    if (_current_position_int32 < 0)
    {
      _current_position_int32 = 0;
      _esp32_encoder->setCount(0);
    }
  }

  int32_t Encoder::get_current_position()
  {
    if (_use_encoder)
    {
      update_position_from_encoder();
    }
    return _current_position_int32;
  }

  uint8_t Encoder::get_normed_current_position()
  {
    if (_use_encoder)
    {
      update_position_from_encoder();
    }

    uint32_t normed_current_position_uint32 =
      (static_cast<uint32_t>(_current_position_int32) * UINT8_MAX) / get_count_drawer_max_extent();

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
    return (position * get_count_drawer_max_extent()) / UINT8_MAX;
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
      _last_timestamp = millis();
    }
  }

  int32_t Encoder::get_integrated_drawer_position(uint32_t active_speed)
  {
    int32_t integrated_position = 0;
    uint32_t current_timestemp = millis();

    integrated_position =
      ((current_timestemp - _last_timestamp) * active_speed) / _config->get_drawer_position_open_loop_integral_gain();

    if (!_is_drawer_moving_out)
    {
      integrated_position *= -1;
    }
    _last_timestamp = current_timestemp;

    return integrated_position;
  }

  uint32_t Encoder::get_count_drawer_max_extent() const
  {
    return _use_encoder ? _config->get_encoder_count_drawer_max_extent()
                        : _config->get_open_loop_count_drawer_max_extent();
  }

}   // namespace motor