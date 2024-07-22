#ifndef DRAWER_CONTROLLER_ENCODER_MONITOR_HPP
#define DRAWER_CONTROLLER_ENCODER_MONITOR_HPP

#include <debug/debug.hpp>
#include <memory>

#include "motor/encoder.hpp"

#define ENCODER_MONITOR_DEFAULT_CHECK_INTERVAL_MS 200

namespace drawer_controller
{
  class EncoderMonitor
  {
   public:
    EncoderMonitor(const std::shared_ptr<Encoder> encoder,
                   const uint16_t encoder_check_interval_ms,
                   const float threshold_in_percent_of_max_extent);

    bool check_if_drawer_is_pushed_in();

   private:
    const std::shared_ptr<Encoder> _encoder;

    const uint16_t _encoder_check_interval_ms;
    const float _threshold_in_percent_of_max_extent;

    int32_t _last_position_int32 = 0;

    uint32_t _last_timestamp_ms = 0;

    bool is_position_update_pending(const uint32_t current_timestamp_ms, const int32_t current_position_uint32);

    void update_position_stamped(const uint32_t current_timestamp_ms, const int32_t current_position_int32);
  };
}   // namespace drawer_controller

#endif   // DRAWER_CONTROLLER_ENCODER_HPP