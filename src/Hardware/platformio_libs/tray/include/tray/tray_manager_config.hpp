#ifndef TRAY_TRAY_CONFIG_HPP
#define TRAY_TRAY_CONFIG_HPP

#include <cstdint>

namespace tray
{
  class TrayManagerConfig
  {
   public:
    TrayManagerConfig() = default;

    void set_speed_deviation_in_percentage_for_stall_when_closing_lid(
      const float speed_deviation_in_percentage_for_stall_when_closing_lid);
    void set_position_offset_for_tray_lid_computation(const uint8_t position_offset_for_tray_lid_computation);
    void set_distance_to_tray_lid_threshold(const uint8_t distance_to_tray_lid_threshold);
    void set_target_speed_to_close_tray_lid(const uint8_t target_speed_to_close_tray_lid);

    float get_speed_deviation_in_percentage_for_stall_when_closing_lid() const;
    uint8_t get_position_offset_for_tray_lid_computation() const;
    uint8_t get_distance_to_tray_lid_threshold() const;
    uint8_t get_target_speed_to_close_tray_lid() const;

   private:
    float _speed_deviation_in_percentage_for_stall_when_closing_lid;
    uint8_t _position_offset_for_tray_lid_computation;
    uint8_t _distance_to_tray_lid_threshold;
    uint8_t _target_speed_to_close_tray_lid;
  };

}   // namespace tray

#endif   // TRAY_TRAY_CONFIG_HPP