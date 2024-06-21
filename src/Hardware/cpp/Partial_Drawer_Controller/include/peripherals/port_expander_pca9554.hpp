#ifndef DRAWER_CONTROLLER_PORT_EXPANDER_PCA9554_HPP
#define DRAWER_CONTROLLER_PORT_EXPANDER_PCA9554_HPP

#include <Arduino.h>
#include <Wire.h>

// PCA9554 Command Byte
#define INPUT_PORT_REGISTER         0x00
#define OUTPUT_PORT_REGISTER        0x01
#define POLARITY_INVERSION_REGISTER 0x02
#define CONFIG_REGISTER             0x03

#define PCA9554_REGISTER_0 0
#define PCA9554_REGISTER_1 1
#define PCA9554_REGISTER_2 2
#define PCA9554_REGISTER_3 3
#define PCA9554_REGISTER_4 4
#define PCA9554_REGISTER_5 5
#define PCA9554_REGISTER_6 6
#define PCA9554_REGISTER_7 7

#define PCA9554_INPUT  0
#define PCA9554_OUTPUT 1

namespace port_expander
{
  class PortExpanderPCA9554
  {
   public:
    PortExpanderPCA9554(byte slave_address)
    {
      _slave_address = slave_address;
    }

    bool twi_read(byte &register_address)
    {
      Wire.beginTransmission(_slave_address);
      Wire.write(register_address);

      if (Wire.endTransmission() == 0)
      {
        delay(15);
        Wire.requestFrom(_slave_address, 1);
        while (Wire.available() < 1)
        {
        }
        register_address = Wire.read();
        return true;
      }
      return false;
    }

    bool twi_write(byte register_address, byte data_write)
    {
      Wire.beginTransmission(_slave_address);
      Wire.write(register_address);
      Wire.write(data_write);

      if (Wire.endTransmission() == 0)
      {
        return true;
      }
      return false;
    };

    bool set_pin_mode(byte pin_number, bool state)
    {
      byte config_register_value = CONFIG_REGISTER;
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
      return twi_write(CONFIG_REGISTER, value);
    }

    bool digital_write(byte pin_number, bool state)
    {
      byte output_register_value = OUTPUT_PORT_REGISTER;
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
      return this->twi_write(OUTPUT_PORT_REGISTER, value);
    }

    bool digital_read(byte &pin_number)
    {
      byte input_register_value = INPUT_PORT_REGISTER;
      if (twi_read(input_register_value) && (pin_number <= 7))
      {
        pin_number = (input_register_value >> pin_number) & 0x01;
        return true;
      }
      return false;
    }

    bool digital_read_port(byte &value)
    {
      value = INPUT_PORT_REGISTER;
      return twi_read(value);
    }

   private:
    uint8_t _slave_address;
  };

}   // namespace port_expander

#endif   // DRAWER_CONTROLLER_PORT_EXPANDER_PCA9554_HPP
