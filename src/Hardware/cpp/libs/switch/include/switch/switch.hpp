#ifndef SWITCH_SWITCH_HPP
#define SWITCH_SWITCH_HPP

#include <memory>

#include "interfaces/i_gpio_wrapper.hpp"

namespace switch_lib
{
  class Switch
  {
  public:
    enum SwitchType
    {
      normally_open,
      normally_closed
    };

  public:
    Switch(const std::shared_ptr<interfaces::IGpioWrapper> gpio_wrapper,
           const uint8_t switch_pin_id,
           const float switch_pressed_threshold,
           const SwitchType switch_type,
           const float weight_new_value);

    bool is_switch_pressed();

    void update_sensor_value();

    float get_moving_average() const;

  private:
    const uint8_t _switch_pin_id;
    const SwitchType _switch_type;
    const std::shared_ptr<interfaces::IGpioWrapper> _gpio_wrapper;
    const float _switch_pressed_threshold;
    const float _weight_new_value;
    float _moving_average;
  };
} // namespace switch_lib

#endif // SWITCH_SWITCH_HPP