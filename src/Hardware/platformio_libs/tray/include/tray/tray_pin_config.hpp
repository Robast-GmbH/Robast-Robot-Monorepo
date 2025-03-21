#ifndef TRAY_TRAY_PIN_CONFIG_HPP
#define TRAY_TRAY_PIN_CONFIG_HPP

namespace tray
{

  struct TrayPinConfig
  {
    uint8_t power_open_pin_id;
    uint8_t power_close_pin_id;
    uint8_t sensor_lock_pin_id;

    // Default Constructor
    TrayPinConfig() : power_open_pin_id(0), power_close_pin_id(0), sensor_lock_pin_id(0)
    {
    }

    // Parameterized Constructor
    TrayPinConfig(const uint8_t power_open_pin_id_input,
                  const uint8_t power_close_pin_id_input,
                  const uint8_t sensor_lock_pin_id_input)
        : power_open_pin_id(power_open_pin_id_input),
          power_close_pin_id(power_close_pin_id_input),
          sensor_lock_pin_id(sensor_lock_pin_id_input)
    {
    }
  };

}   // namespace tray

#endif   // TRAY_TRAY_PIN_CONFIG_HPP