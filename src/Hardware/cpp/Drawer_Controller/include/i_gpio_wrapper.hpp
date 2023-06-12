#ifndef DRAWER_CONTROLLER_I_GPIO_WRAPPER_HPP
#define DRAWER_CONTROLLER_I_GPIO_WRAPPER_HPP

#include <Arduino.h>

namespace drawer_controller
{
  class IGpioWrapper
  {
   public:
    virtual ~IGpioWrapper() = default;

    /**
     * Set the pin mode for the pin mapped to the given pin_mapping_id
     *
     * @param pin_mapping_id mapping ID for the pin whose mode should be set
     * @param state defines whether the pin should be an input or an output pin
     * @return if the digital_read was successfull
     */
    virtual bool set_pin_mode(byte pin_mapping_id, bool state) = 0;

    /**
     * Read the digital input on a Input pin.
     *
     * @param pin_mapping_id mapping ID for the input pin to map the required behaviour
     * @param value pointer to the value which will contain the result of the digital read
     * @return if the digital_read was successfull
     */
    virtual bool digital_read(byte pin_mapping_id, byte &value) = 0;

    /**
     * Write the digital output on a output pin.
     *
     * @param pin_mapping_id mapping ID for the output pin to map the required behaviour
     * @param state target state value of the output
     * @return if the digital_write was successfull
     */
    virtual bool digital_write(byte pin_mapping_id, bool state) = 0;
  };

}   // namespace drawer_controller

#endif   // DRAWER_CONTROLLER_I_GPIO_WRAPPER_HPP