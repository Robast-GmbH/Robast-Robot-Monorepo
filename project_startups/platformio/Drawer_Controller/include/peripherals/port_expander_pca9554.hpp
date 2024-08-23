#ifndef PERIPHERALS_PORT_EXPANDER_PCA9554_HPP
#define PERIPHERALS_PORT_EXPANDER_PCA9554_HPP

#include <Arduino.h>
#include <Wire.h>

namespace port_expander
{
  namespace pca9554
  {
    constexpr uint8_t INPUT_PORT_REGISTER = 0x00;
    constexpr uint8_t OUTPUT_PORT_REGISTER = 0x01;
    constexpr uint8_t POLARITY_INVERSION_REGISTER = 0x02;
    constexpr uint8_t CONFIG_REGISTER = 0x03;

    constexpr uint8_t REGISTER_0 = 0;
    constexpr uint8_t REGISTER_1 = 1;
    constexpr uint8_t REGISTER_2 = 2;
    constexpr uint8_t REGISTER_3 = 3;
    constexpr uint8_t REGISTER_4 = 4;
    constexpr uint8_t REGISTER_5 = 5;
    constexpr uint8_t REGISTER_6 = 6;
    constexpr uint8_t REGISTER_7 = 7;

    constexpr uint8_t PIN_INPUT = 0;
    constexpr uint8_t PIN_OUTPUT = 1;
  }   // namespace pca9554

  class PCA9554
  {
   public:
    PCA9554()
    {
    }

    void attach(std::shared_ptr<TwoWire> wire, byte slave_address)
    {
      _slave_address = slave_address;
      _wire = wire;
    }

    bool twi_read(byte &register_address)
    {
      _wire->beginTransmission(_slave_address);
      _wire->write(register_address);
      if (_wire->endTransmission() == 0)
      {
        _wire->requestFrom(_slave_address, 1);

        while (_wire->available() < 1)
        {
        }
        register_address = _wire->read();
        return true;
      }
      return false;
    }

    bool twi_write(byte register_address, byte data_write)
    {
      _wire->beginTransmission(_slave_address);
      _wire->write(register_address);
      _wire->write(data_write);

      if (_wire->endTransmission() == 0)
      {
        return true;
      }
      return false;
    };

    bool set_pin_mode(byte pin_number, bool state)
    {
      byte config_register_value = pca9554::CONFIG_REGISTER;
      if (twi_read(config_register_value) && (pin_number <= 7))
      {
        if (!state)
        {
          config_register_value |= (1 << pin_number);
          if (port_mode(config_register_value))
          {
            return true;
          }
          return false;
        }
        else
        {
          config_register_value &= ~(1 << pin_number);
          if (port_mode(config_register_value))
          {
            return true;
          }
          return false;
        }
      }
      return false;
    }

    bool port_mode(byte value)
    {
      return twi_write(pca9554::CONFIG_REGISTER, value);
    }

    bool digital_write(byte pin_number, bool state)
    {
      byte output_register_value = pca9554::OUTPUT_PORT_REGISTER;
      if (twi_read(output_register_value) && pin_number <= 7)
      {
        if (state)
        {
          output_register_value |= (1 << pin_number);
          if (digital_write_port(output_register_value))
          {
            return true;
          }
          return false;
        }
        else if (!state)
        {
          output_register_value &= ~(1 << pin_number);
          if (digital_write_port(output_register_value))
          {
            return true;
          }
          return false;
        }
      }
      return false;
    }

    bool digital_write_port(byte value)
    {
      return this->twi_write(pca9554::OUTPUT_PORT_REGISTER, value);
    }

    bool digital_read(byte &pin_number)
    {
      byte input_register_value = pca9554::INPUT_PORT_REGISTER;
      if (twi_read(input_register_value) && (pin_number <= 7))
      {
        pin_number = (input_register_value >> pin_number) & 0x01;
        return true;
      }
      return false;
    }

    bool digital_read_port(byte &value)
    {
      value = pca9554::INPUT_PORT_REGISTER;
      return twi_read(value);
    }

   private:
    uint8_t _slave_address;
    std::shared_ptr<TwoWire> _wire;
  };

}   // namespace port_expander

#endif   // PERIPHERALS_PORT_EXPANDER_PCA9554_HPP