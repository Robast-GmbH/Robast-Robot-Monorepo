#include "switch/switch.hpp"

namespace switch_lib
{
  Switch::Switch(const std::shared_ptr<interfaces::IGpioWrapper> gpio_wrapper,
                 const uint8_t pin_id,
                 const float switch_pressed_threshold,
                 const SwitchType type,
                 const float new_reading_weight)
      : _gpio_wrapper{gpio_wrapper},
        _pin_id{pin_id},
        _switch_pressed_threshold{switch_pressed_threshold},
        _type{type},
        _new_reading_weight{new_reading_weight}
  {
    _gpio_wrapper->set_pin_mode(_pin_id, true);
  }

  bool Switch::is_switch_pressed()
  {
    if (_type == normally_open)
    {
      return _moving_average > _switch_pressed_threshold;
    }
    return _moving_average < (1 - _switch_pressed_threshold);
  }

  void Switch::update_sensor_value()
  {
    uint8_t digital_read_result;
    _gpio_wrapper->digital_read(_pin_id, digital_read_result);
    _moving_average = _new_reading_weight * digital_read_result + (1 - _new_reading_weight) * _moving_average;
  }
}   // namespace switch_lib