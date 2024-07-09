#ifndef DRAWER_CONTROLLER_I_GPIO_WRAPPER_HPP
#define DRAWER_CONTROLLER_I_GPIO_WRAPPER_HPP

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
    virtual bool set_pin_mode(const uint8_t pin_mapping_id, const bool state) const = 0;

    /**
     * Read the digital input on a Input pin.
     *
     * @param pin_mapping_id mapping ID for the input pin to map the required behaviour
     * @param value pointer to the value which will contain the result of the digital read
     * @return if the digital_read was successfull
     */
    virtual bool digital_read(const uint8_t pin_mapping_id, uint8_t &value) const = 0;

    /**
     * Write the digital output on a output pin.
     *
     * @param pin_mapping_id mapping ID for the output pin to map the required behaviour
     * @param state target state value of the output
     * @return if the digital_write was successfull
     */
    virtual bool digital_write(const uint8_t pin_mapping_id, const bool state) const = 0;

    /**
     * If you want to set the pin mode of a GPIO, this function returns the OUTPUT pin mode value for it
     *
     * @return OUTPUT pin mode value
     */
    virtual bool get_gpio_output_pin_mode() const = 0;

    /**
     * If you want to set the pin mode of a GPIO, this function returns the INPUT pin mode value for it
     *
     * @return INPUT pin mode value
     */
    virtual bool get_gpio_input_pin_mode() const = 0;

    /**
     * Returns the GPIO number for the given pin ID
     *
     * @param pin_id the pin ID for which the GPIO number should be returned
     * @return the GPIO number for the given pin ID
     */
    virtual uint8_t get_gpio_num_for_pin_id(const uint8_t pin_id) const = 0;
  };

} // namespace drawer_controller

#endif // DRAWER_CONTROLLER_I_GPIO_WRAPPER_HPP