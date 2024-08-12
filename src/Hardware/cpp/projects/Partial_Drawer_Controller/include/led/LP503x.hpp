#ifndef PARTIAL_DRAWER_CONTROLLER_LP503x_HPP
#define PARTIAL_DRAWER_CONTROLLER_LP503x_HPP

#include <Arduino.h>
#include <Wire.h>

namespace lp503x
{

  namespace register_address
  {
    // Register definitions
    constexpr byte DEVICE_CONFIG0 = 0x00;    // Chip_EN
    constexpr byte DEVICE_CONFIG1 = 0x01;    // Configurations for Log_scale, Power_save, Auto_inc, PWM_dithering,
                                             // Max_current_option and LED_Global_off
    constexpr byte LED_CONFIG0 = 0x02;       // Contains the BANK configurations for LED0_BANK_EN - LED7_BANK_EN
    constexpr byte LED_CONFIG1 = 0x03;       // Contains the BANK configurations for LED8_BANK_EN - LED11_BANK_EN
    constexpr byte BANK_BRIGHTNESS = 0x04;   // Contains the BANK brightness level
    constexpr byte BANK_A_COLOR = 0x05;      // Contains the BANK A color value
    constexpr byte BANK_B_COLOR = 0x06;      // Contains the BANK B color value
    constexpr byte BANK_C_COLOR = 0x07;      // Contains the BANK C color value

    constexpr byte LED0_BRIGHTNESS = 0x08;    // Contains the brightness level for LED 0
    constexpr byte LED1_BRIGHTNESS = 0x09;    // Contains the brightness level for LED 1
    constexpr byte LED2_BRIGHTNESS = 0x0A;    // Contains the brightness level for LED 2
    constexpr byte LED3_BRIGHTNESS = 0x0B;    // Contains the brightness level for LED 3
    constexpr byte LED4_BRIGHTNESS = 0x0C;    // Contains the brightness level for LED 4
    constexpr byte LED5_BRIGHTNESS = 0x0D;    // Contains the brightness level for LED 5
    constexpr byte LED6_BRIGHTNESS = 0x0E;    // Contains the brightness level for LED 6
    constexpr byte LED7_BRIGHTNESS = 0x0F;    // Contains the brightness level for LED 7
    constexpr byte LED8_BRIGHTNESS = 0x10;    // Contains the brightness level for LED 8
    constexpr byte LED9_BRIGHTNESS = 0x11;    // Contains the brightness level for LED 9
    constexpr byte LED10_BRIGHTNESS = 0x12;   // Contains the brightness level for LED 10
    constexpr byte LED11_BRIGHTNESS = 0x13;   // Contains the brightness level for LED 11

    constexpr byte OUT0_COLOR = 0x14;    // Contains the color value for output 0
    constexpr byte OUT1_COLOR = 0x15;    // Contains the color value for output 1
    constexpr byte OUT2_COLOR = 0x16;    // Contains the color value for output 2
    constexpr byte OUT3_COLOR = 0x17;    // Contains the color value for output 3
    constexpr byte OUT4_COLOR = 0x18;    // Contains the color value for output 4
    constexpr byte OUT5_COLOR = 0x19;    // Contains the color value for output 5
    constexpr byte OUT6_COLOR = 0x1A;    // Contains the color value for output 6
    constexpr byte OUT7_COLOR = 0x1B;    // Contains the color value for output 7
    constexpr byte OUT8_COLOR = 0x1C;    // Contains the color value for output 8
    constexpr byte OUT9_COLOR = 0x1D;    // Contains the color value for output 9
    constexpr byte OUT10_COLOR = 0x1E;   // Contains the color value for output 10
    constexpr byte OUT11_COLOR = 0x1F;   // Contains the color value for output 11
    constexpr byte OUT12_COLOR = 0x20;   // Contains the color value for output 12
    constexpr byte OUT13_COLOR = 0x21;   // Contains the color value for output 13
    constexpr byte OUT14_COLOR = 0x22;   // Contains the color value for output 14
    constexpr byte OUT15_COLOR = 0x23;   // Contains the color value for output 15
    constexpr byte OUT16_COLOR = 0x24;   // Contains the color value for output 16
    constexpr byte OUT17_COLOR = 0x25;   // Contains the color value for output 17
    constexpr byte OUT18_COLOR = 0x26;   // Contains the color value for output 18
    constexpr byte OUT19_COLOR = 0x27;   // Contains the color value for output 19
    constexpr byte OUT20_COLOR = 0x28;   // Contains the color value for output 20
    constexpr byte OUT21_COLOR = 0x29;   // Contains the color value for output 21
    constexpr byte OUT22_COLOR = 0x2A;   // Contains the color value for output 22
    constexpr byte OUT23_COLOR = 0x2B;   // Contains the color value for output 23
    constexpr byte OUT24_COLOR = 0x2C;   // Contains the color value for output 24
    constexpr byte OUT25_COLOR = 0x2D;   // Contains the color value for output 25
    constexpr byte OUT26_COLOR = 0x2E;   // Contains the color value for output 26
    constexpr byte OUT27_COLOR = 0x2F;   // Contains the color value for output 27
    constexpr byte OUT28_COLOR = 0x30;   // Contains the color value for output 28
    constexpr byte OUT29_COLOR = 0x31;   // Contains the color value for output 29
    constexpr byte OUT30_COLOR = 0x32;   // Contains the color value for output 30
    constexpr byte OUT31_COLOR = 0x33;   // Contains the color value for output 31
    constexpr byte OUT32_COLOR = 0x34;   // Contains the color value for output 32
    constexpr byte OUT33_COLOR = 0x35;   // Contains the color value for output 33
    constexpr byte OUT34_COLOR = 0x36;   // Contains the color value for output 34
    constexpr byte OUT35_COLOR = 0x37;   // Contains the color value for output 35

    constexpr byte RESET_REGISTERS = 0x38;   // Resets all the registers to their default values
  }   // namespace register_address

  class LP503x
  {
   public:
    LP503x(byte slave_address, std::shared_ptr<TwoWire> wire) : _slave_address{slave_address}, _wire{wire}
    {
    }

    bool twi_read(byte &register_address, uint8_t &data)
    {
      _wire->beginTransmission(_slave_address);
      _wire->write(register_address);

      if (_wire->endTransmission(false) == 0)
      {
        delay(15);
        _wire->requestFrom(_slave_address, 1);
        while (_wire->available() < 1)
        {
        }
        data = _wire->read();
        return true;
      }
      return false;
    }

    bool twi_write(byte register_address, byte data_write)
    {
      _wire->beginTransmission(_slave_address);
      _wire->write(register_address);
      _wire->write(data_write);

      return _wire->endTransmission() == 0;
    }

    bool set_led_brigthness(byte led_number, byte brightness)
    {
      byte register_address = lp503x::register_address::LED0_BRIGHTNESS + led_number;
      return twi_write(register_address, brightness);
    }

    bool set_led_output_color_by_number(byte led_number, byte color)
    {
      byte register_address = lp503x::register_address::OUT0_COLOR + led_number;
      return twi_write(register_address, color);
    }

    bool set_led_output_color_by_register(byte register_address, byte color)
    {
      return twi_write(register_address, color);
    }

    bool reset_registers()
    {
      return twi_write(lp503x::register_address::RESET_REGISTERS, 0xFF);
    }

    bool enable_chip()
    {
      return twi_write(lp503x::register_address::DEVICE_CONFIG0, 0b01000000);
    }

    bool disable_chip()
    {
      return twi_write(lp503x::register_address::DEVICE_CONFIG0, 0b00000000);
    }

    bool read_register(byte register_address, byte &data_read)
    {
      return twi_read(register_address, data_read);
    }

   private:
    std::shared_ptr<TwoWire> _wire;
    byte _slave_address;
  };
}   // namespace lp503x

#endif   // PARTIAL_DRAWER_CONTROLLER_LP503x_HPP