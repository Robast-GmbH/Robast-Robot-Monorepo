#ifndef MOTOR_ENCODER_MONITOR_HPP
#define MOTOR_ENCODER_MONITOR_HPP

#include <debug/debug.hpp>
#include <memory>

#include "motor/encoder.hpp"
#include "motor/enconder_config.hpp"

namespace motor
{
  class EncoderMonitor
  {
   public:
    EncoderMonitor(const std::shared_ptr<Encoder> encoder, const std::shared_ptr<EncoderConfig> config);

    bool check_if_drawer_is_pushed_in();

    bool check_if_drawer_is_pulled_out();

   private:
    const std::shared_ptr<Encoder> _encoder;
    const std::shared_ptr<EncoderConfig> _config;

    static constexpr uint16_t _DEFAULT_ENCODER_MONITOR_CHECK_INTERVAL_MS = 200;

    int32_t _last_position_int32 = 0;

    uint32_t _last_timestamp_ms = 0;

    bool is_position_update_pending(const uint32_t current_timestamp_ms, const int32_t current_position_uint32);

    void update_position_stamped(const uint32_t current_timestamp_ms, const int32_t current_position_int32);

    bool check_if_drawer_is_moved(const uint32_t check_interval_ms,
                                  const int32_t threshold_in_percent_of_max_extent,
                                  const bool is_push_in);
  };
}   // namespace motor

#endif   // MOTOR_ENCODER_MONITOR_HPP