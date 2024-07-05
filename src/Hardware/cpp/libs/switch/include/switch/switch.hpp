#ifndef DRAWER_CONTROLLER_SWITCH_HPP
#define DRAWER_CONTROLLER_SWITCH_HPP

#include <memory>

#include "interfaces/i_gpio_wrapper.hpp"

namespace drawer_controller
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
    Switch(std::shared_ptr<IGpioWrapper> gpio_wrapper,
           uint8_t switch_pin_id,
           float switch_pressed_threshold,
           SwitchType switch_type);

    bool is_switch_pressed();

    void update_sensor_value();

   private:
    uint8_t _switch_pin;
    SwitchType _switch_type;
    std::shared_ptr<IGpioWrapper> _gpio_wrapper;
    float _switch_pressed_threshold;
    float _moving_average;
  };
}   // namespace drawer_controller

#endif   // DRAWER_CONTROLLER_SWITCH_HPP