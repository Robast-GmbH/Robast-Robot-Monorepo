#include "peripherals/switch.hpp"

namespace drawer_controller
{
  Switch::Switch(std::shared_ptr<IGpioWrapper> gpio_wrapper,
                 uint8_t switch_pin_id,
                 float switch_pressed_threshold,
                 SwitchType switch_type)
      : _gpio_wrapper{gpio_wrapper},
        _switch_pin{switch_pin_id},
        _switch_pressed_threshold{switch_pressed_threshold},
        _switch_type{switch_type},
        _moving_average{0.0}
  {
  }

  bool Switch::is_switch_pressed()
  {
    if (_switch_type == normally_open)
    {
      return _moving_average < (1 - _switch_pressed_threshold);
    }
    return _moving_average > _switch_pressed_threshold;
  }

  void Switch::update_sensor_value()
  {
    byte digital_read_result;
    _gpio_wrapper->digital_read(_switch_pin, digital_read_result);
    _moving_average = 0.2 * digital_read_result + 0.8 * _moving_average;
  }
}   // namespace drawer_controller