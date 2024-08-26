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
           const SwitchType type,
           const float new_reading_weight);

    bool is_switch_pressed();

    void update_sensor_value();

  private:
    const uint8_t _pin_id;
    const SwitchType _type;
    const std::shared_ptr<interfaces::IGpioWrapper> _gpio_wrapper;
    const float _switch_pressed_threshold;
    const float _new_reading_weight;
    float _moving_average = 0;
  };
} // namespace switch_lib

#endif // SWITCH_SWITCH_HPP