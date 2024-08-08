#include "motor/encoder_monitor.hpp"

namespace motor
{
  EncoderMonitor::EncoderMonitor(const std::shared_ptr<Encoder> encoder, const std::shared_ptr<EncoderConfig> config)
      : _encoder(encoder), _config(config)
  {
  }

  bool EncoderMonitor::check_if_drawer_is_pushed_in()
  {
    uint32_t current_timestamp_ms = millis();

    int32_t current_position_int32 = _encoder->get_current_position();

    if (is_position_update_pending(current_timestamp_ms, current_position_int32))
    {
      return false;
    }

    if (current_timestamp_ms - _last_timestamp_ms > _config->get_drawer_push_in_encoder_check_interval_ms())
    {
      int32_t position_difference = _last_position_int32 - current_position_int32;
      int32_t threshold_for_pos_diff =
        _config->get_drawer_push_in_threshold_in_percent_of_max_extent() * _encoder->get_count_drawer_max_extent();

      if (position_difference > threshold_for_pos_diff)
      {
        debug_printf(
          "[EncoderMonitor]: Position difference %d is bigger than threshold %d! Current Position %d, Last Position "
          "%d\n",
          position_difference,
          threshold_for_pos_diff,
          current_position_int32,
          _last_position_int32);
        return true;
      }
      else
      {
        update_position_stamped(current_timestamp_ms, current_position_int32);
        return false;
      }
    }
  }

  bool EncoderMonitor::is_position_update_pending(const uint32_t current_timestamp_ms,
                                                  const int32_t current_position_int32)
  {
    // In case the postion has not been updated (e.g. when the drawer is intentionally moving)
    if (current_timestamp_ms - _last_timestamp_ms >
        _config->get_drawer_push_in_encoder_check_interval_ms() + DEFAULT_ENCODER_MONITOR_CHECK_INTERVAL_MS)
    {
      update_position_stamped(current_timestamp_ms, current_position_int32);
      return true;
    }
    return false;
  }

  void EncoderMonitor::update_position_stamped(const uint32_t current_timestamp_ms,
                                               const int32_t current_position_int32)
  {
    _last_timestamp_ms = current_timestamp_ms;
    _last_position_int32 = current_position_int32;
  }
}   // namespace motor
