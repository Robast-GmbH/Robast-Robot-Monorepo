#ifndef GPIO_GPIO_WRAPPER_PCA9554_HPP
#define GPIO_GPIO_WRAPPER_PCA9554_HPP

#include <memory>

#include "gpio/gpio_info.hpp"
#include "interfaces/i_gpio_wrapper.hpp"
#include "peripherals/port_expander_pca9554.hpp"

namespace gpio
{
  using slave_address_by_register = std::tuple<uint8_t, uint8_t>;

  constexpr bool FOUND_PIN_INFO = true;
  constexpr bool PIN_INFO_NOT_FOUND = false;

  class GpioWrapperPca9554 : public interfaces::IGpioWrapper
  {
   public:
    GpioWrapperPca9554(
      const std::shared_ptr<TwoWire> wire,
      const std::unordered_map<uint8_t, std::shared_ptr<port_expander::PCA9554>> slave_address_to_port_expander,
      const std::unordered_map<uint8_t, gpio::GpioInfo> pin_mapping_id_to_gpio_info,
      const std::unordered_map<uint8_t, slave_address_by_register> pin_mapping_id_to_slave_address_by_register)
        : _wire{wire},
          _slave_address_to_port_expander{slave_address_to_port_expander},
          _pin_mapping_id_to_gpio_info{pin_mapping_id_to_gpio_info},
          _pin_mapping_id_to_slave_address_by_register{pin_mapping_id_to_slave_address_by_register}
    {
      for (auto &slave_address_port_expander_tuple : slave_address_to_port_expander)
      {
        std::shared_ptr<port_expander::PCA9554> port_expander = slave_address_port_expander_tuple.second;
        uint8_t slave_address = slave_address_port_expander_tuple.first;

        port_expander->attach(_wire, slave_address);
      }
    }

    /**
     * Set the pin mode for the pin mapped to the given pin_mapping_id
     *
     * @param pin_mapping_id mapping ID for the pin whose mode should be set
     * @param is_input defines whether the pin should be an input or an output pin
     * @return if the set_pin_mode was successfull
     */
    bool set_pin_mode(const byte pin_mapping_id, const bool is_input) const
    {
      // check if requested pin is accessible via port expander
      auto [found_pin_info, slave_address, register_id] = get_pin_info_for_port_expander(pin_mapping_id);
      if (found_pin_info)
      {
        return _slave_address_to_port_expander.at(slave_address)
          ->set_pin_mode(register_id,
                         is_input ? port_expander::pca9554::PIN_INPUT : port_expander::pca9554::PIN_OUTPUT);
      }

      // check if requested pin is accessible via GPIO from the microcontroller
      if (_pin_mapping_id_to_gpio_info.find(pin_mapping_id) != _pin_mapping_id_to_gpio_info.end())
      {
        if (_pin_mapping_id_to_gpio_info.at(pin_mapping_id).is_input && !is_input)
        {
          Serial.printf("Error! Trying to set pin mode for pin id %d to input, but it is input only!\n",
                        pin_mapping_id);
          return false;
        }

        pinMode(_pin_mapping_id_to_gpio_info.at(pin_mapping_id).pin_number, is_input ? INPUT : OUTPUT);
        return true;
      }

      Serial.printf("Error! Pin mapping ID %d not found when setting pin mode!\n", pin_mapping_id);
      return false;
    }

    /**
     * Read the digital input on a Input pin.
     *
     * @param pin_mapping_id mapping ID for the input pin to map the required behaviour
     * @param value pointer to the value which will contain the result of the digital read
     * @return if the digital_read was successfull
     */
    bool digital_read(const byte pin_mapping_id, byte &value) const
    {
      auto [found_pin_info, slave_address, register_id] = get_pin_info_for_port_expander(pin_mapping_id);

      if (found_pin_info)
      {
        value = register_id;
        _slave_address_to_port_expander.at(slave_address)->digital_read(value);

        return true;
      }

      if (_pin_mapping_id_to_gpio_info.find(pin_mapping_id) != _pin_mapping_id_to_gpio_info.end())
      {
        value = digitalRead(_pin_mapping_id_to_gpio_info.at(pin_mapping_id).pin_number);
        return true;
      }

      Serial.printf("Error! Pin mapping ID %d not found when trying to digital_read!\n", pin_mapping_id);
      return false;
    }

    /**
     * Write the digital output on a output pin.
     *
     * @param pin_mapping_id mapping ID for the output pin to map the required behaviour
     * @param state target state value of the output
     * @return if the digital_write was successfull
     */
    bool digital_write(const byte pin_mapping_id, const bool state) const
    {
      // use get_info_for_port_expander(pin_mapping_id);
      auto [found_pin_info, slave_address, register_id] = get_pin_info_for_port_expander(pin_mapping_id);
      if (found_pin_info)
      {
        return _slave_address_to_port_expander.at(slave_address)->digital_write(register_id, state ? HIGH : LOW);
      }

      if (_pin_mapping_id_to_gpio_info.find(pin_mapping_id) != _pin_mapping_id_to_gpio_info.end())
      {
        digitalWrite(_pin_mapping_id_to_gpio_info.at(pin_mapping_id).pin_number, state);
        return true;
      }

      Serial.printf("Error! Pin mapping ID %d not found when trying to digital_write!\n", pin_mapping_id);
      return false;
    }

    uint8_t get_gpio_num_for_pin_id(const uint8_t pin_id) const
    {
      if (_pin_mapping_id_to_gpio_info.find(pin_id) == _pin_mapping_id_to_gpio_info.end())
      {
        Serial.printf("Error! Pin mapping ID %d not found when trying to get GPIO number!\n", pin_id);
        return 0;
      }

      return _pin_mapping_id_to_gpio_info.at(pin_id).pin_number;
    }

   private:
    const std::shared_ptr<TwoWire> _wire;

    const std::unordered_map<uint8_t, gpio::GpioInfo> _pin_mapping_id_to_gpio_info;

    const std::unordered_map<uint8_t, std::shared_ptr<port_expander::PCA9554>> _slave_address_to_port_expander;

    const std::unordered_map<uint8_t, slave_address_by_register> _pin_mapping_id_to_slave_address_by_register;

    std::tuple<bool, uint8_t, uint8_t> get_pin_info_for_port_expander(uint8_t pin_mapping_id) const
    {
      if (_pin_mapping_id_to_slave_address_by_register.find(pin_mapping_id) !=
          _pin_mapping_id_to_slave_address_by_register.end())
      {
        auto [slave_address, register_id] = _pin_mapping_id_to_slave_address_by_register.at(pin_mapping_id);

        return std::make_tuple(FOUND_PIN_INFO, slave_address, register_id);
      }
      return std::make_tuple(PIN_INFO_NOT_FOUND, 0, 0);   // Default values if not found
    }
  };

}   // namespace gpio
#endif   // GPIO_GPIO_WRAPPER_PCA9554_HPP
