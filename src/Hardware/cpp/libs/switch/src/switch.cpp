#include "switch/switch.hpp"

namespace switch_ns
{
  Switch::Switch(const std::shared_ptr<interfaces::IGpioWrapper> gpio_wrapper,
                 const uint8_t switch_pin_id,
                 const float switch_pressed_threshold,
                 const SwitchType switch_type,
                 const float weight_new_value)
      : _gpio_wrapper{gpio_wrapper},
        _switch_pin_id{switch_pin_id},
        _switch_pressed_threshold{switch_pressed_threshold},
        _switch_type{switch_type},
        _weight_new_value{weight_new_value},
        _moving_average{0.0}
  {
    _gpio_wrapper->set_pin_mode(_switch_pin_id, true);
  }

  bool Switch::is_switch_pressed()
  {
    if (_switch_type == normally_open)
    {
      return _moving_average > _switch_pressed_threshold;
    }
    return _moving_average < (1 - _switch_pressed_threshold);
  }

  void Switch::update_sensor_value()
  {
    uint8_t digital_read_result;
    _gpio_wrapper->digital_read(_switch_pin_id, digital_read_result);
    _moving_average = _weight_new_value * digital_read_result + (1 - _weight_new_value) * _moving_average;
  }

  float Switch::get_moving_average() const
  {
    return _moving_average;
  }
}   // namespace switch_ns