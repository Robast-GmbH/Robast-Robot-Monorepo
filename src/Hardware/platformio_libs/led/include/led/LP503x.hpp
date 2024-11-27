#ifndef LED_LP503x_HPP
#define LED_LP503x_HPP

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

    constexpr byte LED_0_BRIGHTNESS = 0x08;
    constexpr byte LED_1_BRIGHTNESS = 0x09;
    constexpr byte LED_2_BRIGHTNESS = 0x0A;
    constexpr byte LED_3_BRIGHTNESS = 0x0B;
    constexpr byte LED_4_BRIGHTNESS = 0x0C;
    constexpr byte LED_5_BRIGHTNESS = 0x0D;
    constexpr byte LED_6_BRIGHTNESS = 0x0E;
    constexpr byte LED_7_BRIGHTNESS = 0x0F;
    constexpr byte LED_8_BRIGHTNESS = 0x10;
    constexpr byte LED_9_BRIGHTNESS = 0x11;
    constexpr byte LED_10_BRIGHTNESS = 0x12;
    constexpr byte LED_11_BRIGHTNESS = 0x13;

    constexpr byte OUT_0_COLOR = 0x14;
    constexpr byte OUT_1_COLOR = 0x15;
    constexpr byte OUT_2_COLOR = 0x16;
    constexpr byte OUT_3_COLOR = 0x17;
    constexpr byte OUT_4_COLOR = 0x18;
    constexpr byte OUT_5_COLOR = 0x19;
    constexpr byte OUT_6_COLOR = 0x1A;
    constexpr byte OUT_7_COLOR = 0x1B;
    constexpr byte OUT_8_COLOR = 0x1C;
    constexpr byte OUT_9_COLOR = 0x1D;
    constexpr byte OUT_10_COLOR = 0x1E;
    constexpr byte OUT_11_COLOR = 0x1F;
    constexpr byte OUT_12_COLOR = 0x20;
    constexpr byte OUT_13_COLOR = 0x21;
    constexpr byte OUT_14_COLOR = 0x22;
    constexpr byte OUT_15_COLOR = 0x23;
    constexpr byte OUT_16_COLOR = 0x24;
    constexpr byte OUT_17_COLOR = 0x25;
    constexpr byte OUT_18_COLOR = 0x26;
    constexpr byte OUT_19_COLOR = 0x27;
    constexpr byte OUT_20_COLOR = 0x28;
    constexpr byte OUT_21_COLOR = 0x29;
    constexpr byte OUT_22_COLOR = 0x2A;
    constexpr byte OUT_23_COLOR = 0x2B;
    constexpr byte OUT_24_COLOR = 0x2C;
    constexpr byte OUT_25_COLOR = 0x2D;
    constexpr byte OUT_26_COLOR = 0x2E;
    constexpr byte OUT_27_COLOR = 0x2F;
    constexpr byte OUT_28_COLOR = 0x30;
    constexpr byte OUT_29_COLOR = 0x31;
    constexpr byte OUT_30_COLOR = 0x32;
    constexpr byte OUT_31_COLOR = 0x33;
    constexpr byte OUT_32_COLOR = 0x34;
    constexpr byte OUT_33_COLOR = 0x35;
    constexpr byte OUT_34_COLOR = 0x36;
    constexpr byte OUT_35_COLOR = 0x37;

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
      byte register_address = lp503x::register_address::LED_0_BRIGHTNESS + led_number;
      return twi_write(register_address, brightness);
    }

    bool set_led_output_color_by_number(byte led_number, byte color)
    {
      byte register_address = lp503x::register_address::OUT_0_COLOR + led_number;
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

#endif   // LED_LP503x_HPP