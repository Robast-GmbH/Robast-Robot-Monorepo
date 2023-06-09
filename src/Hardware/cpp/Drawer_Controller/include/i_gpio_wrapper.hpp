#ifndef DRAWER_CONTROLLER_I_GPIO_WRAPPER_HPP
#define DRAWER_CONTROLLER_I_GPIO_WRAPPER_HPP

#include <Arduino.h>

namespace drawer_controller
{
  class IGpioWrapper
  {
   public:
    virtual ~IGpioWrapper() = default;
    virtual bool set_pin_mode(byte pin_mapping_id, bool state) = 0;
    virtual bool digital_read(byte pin_mapping_id, byte &value) = 0;
    virtual bool digital_write(byte pin_mapping_id, bool state) = 0;
  };

}   // namespace drawer_controller

#endif   // DRAWER_CONTROLLER_I_GPIO_WRAPPER_HPP