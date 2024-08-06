#ifndef DRAWER_CONTROLLER_GPIO_WRAPPER_PCA9535_HPP
#define DRAWER_CONTROLLER_GPIO_WRAPPER_PCA9535_HPP

#include <Arduino.h>
#include <PCA95x5.h>
#include <Wire.h>

#include "gpio/gpio_info.hpp"
#include "interfaces/i_gpio_wrapper.hpp"
#include "peripherals/pinout_defines.hpp"

namespace drawer_controller
{
  using port_info = std::tuple<uint8_t, PCA95x5::Port::Port>;

  class GpioWrapperPca9535 : public IGpioWrapper
  {
   public:
    GpioWrapperPca9535(const std::shared_ptr<TwoWire> wire,
                       const std::unordered_map<uint8_t, std::shared_ptr<PCA9535>> slave_address_to_port_expander,
                       const std::unordered_map<uint8_t, gpio_info::GpioInfo> pin_mapping_id_to_gpio_info,
                       const std::unordered_map<uint8_t, port_info> pin_mapping_id_to_port)
        : _wire{wire},
          _slave_address_to_port_expander{slave_address_to_port_expander},
          _pin_mapping_id_to_gpio_info{pin_mapping_id_to_gpio_info},
          _pin_mapping_id_to_port{pin_mapping_id_to_port}
    {
      for (auto &slave_address_port_expander_tuple : slave_address_to_port_expander)
      {
        std::shared_ptr<PCA9535> port_expander = slave_address_port_expander_tuple.second;
        uint8_t slave_address = slave_address_port_expander_tuple.first;

        port_expander->attach(*_wire, slave_address);
        port_expander->polarity(PCA95x5::Polarity::ORIGINAL_ALL);
      }
    }

    /**
     * Set the pin mode for the pin mapped to the given pin_mapping_id
     *
     * @param pin_mapping_id mapping ID for the pin whose mode should be set
     * @param is_input defines whether the pin should be an input or an output pin
     * @return if the digital_read was successfull
     */
    bool set_pin_mode(const byte pin_mapping_id, const bool is_input) const
    {
      // check if requested pin is accessible via port expander
      if (_pin_mapping_id_to_port.find(pin_mapping_id) != _pin_mapping_id_to_port.end())
      {
        port_info port_info = _pin_mapping_id_to_port.at(pin_mapping_id);

        uint8_t port_expander_id = std::get<0>(port_info);
        PCA95x5::Port::Port port_id = std::get<1>(port_info);

        return _slave_address_to_port_expander.at(port_expander_id)
          ->direction(port_id, (is_input == PCA95x5::Direction::IN) ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
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
      if (_pin_mapping_id_to_port.find(pin_mapping_id) == _pin_mapping_id_to_port.end())
      {
        if (_pin_mapping_id_to_gpio_info.find(pin_mapping_id) == _pin_mapping_id_to_gpio_info.end())
        {
          Serial.printf("Error! Pin mapping ID %d not found when trying to digital_read!\n", pin_mapping_id);
          return false;
        }

        value = digitalRead(_pin_mapping_id_to_gpio_info.at(pin_mapping_id).pin_number);
        return true;
      }

      port_info port_info = _pin_mapping_id_to_port.at(pin_mapping_id);
      uint8_t port_expander_id = std::get<0>(port_info);
      PCA95x5::Port::Port port_id = std::get<1>(port_info);

      value = _slave_address_to_port_expander.at(port_expander_id)->read(port_id);
      return true;
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
      if (_pin_mapping_id_to_port.find(pin_mapping_id) == _pin_mapping_id_to_port.end())
      {
        if (_pin_mapping_id_to_gpio_info.find(pin_mapping_id) == _pin_mapping_id_to_gpio_info.end())
        {
          Serial.printf("Error! Pin mapping ID %d not found when trying to digital_write!\n", pin_mapping_id);
          return false;
        }

        digitalWrite(_pin_mapping_id_to_gpio_info.at(pin_mapping_id).pin_number, state);
        return true;
      }

      port_info port_info = _pin_mapping_id_to_port.at(pin_mapping_id);
      uint8_t port_expander_id = std::get<0>(port_info);
      PCA95x5::Port::Port port_id = std::get<1>(port_info);

      return _slave_address_to_port_expander.at(port_expander_id)
        ->write(port_id, state ? PCA95x5::Level::H : PCA95x5::Level::L);
    }

    bool get_gpio_output_pin_mode() const
    {
      return PCA95x5::Direction::OUT;
    }

    bool get_gpio_input_pin_mode() const
    {
      return PCA95x5::Direction::IN;
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

    const std::unordered_map<uint8_t, gpio_info::GpioInfo> _pin_mapping_id_to_gpio_info;

    const std::unordered_map<uint8_t, std::shared_ptr<PCA9535>> _slave_address_to_port_expander;

    const std::unordered_map<uint8_t, port_info> _pin_mapping_id_to_port;
  };

}   // namespace drawer_controller
#endif   // DRAWER_CONTROLLER_GPIO_WRAPPER_PCA9535_HPP
