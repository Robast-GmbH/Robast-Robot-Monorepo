#ifndef PARTIAL_DRAWER_CONTROLLER_LP503x_HPP
#define PARTIAL_DRAWER_CONTROLLER_LP503x_HPP

#include <Arduino.h>
#include <Wire.h>

namespace lp503x
{

// Register definitions
#define DEVICE_CONFIG0 0x00   // Chip_EN
#define DEVICE_CONFIG1 \
  0x01   // Configurations for Log_scale, Power_save, Auto_inc, PWM_dithering, Max_current_option and LED_Global_off
#define LED_CONFIG0     0x02   // Contains the BANK configurations for LED0_BANK_EN - LED7_BANK_EN
#define LED_CONFIG1     0x03   // Contains the BANK configurations for LED8_BANK_EN - LED11_BANK_EN
#define BANK_BRIGHTNESS 0x04   // Contains the BANK brightness level
#define BANK_A_COLOR    0x05   // Contains the BANK A color value
#define BANK_B_COLOR    0x06   // Contains the BANK B color value
#define BANK_C_COLOR    0x07   // Contains the BANK C color value

#define LED0_BRIGHTNESS  0x08   // Contains the brightness level for LED 0
#define LED1_BRIGHTNESS  0x09   // Contains the brightness level for LED 1
#define LED2_BRIGHTNESS  0x0A   // Contains the brightness level for LED 2
#define LED3_BRIGHTNESS  0x0B   // Contains the brightness level for LED 3
#define LED4_BRIGHTNESS  0x0C   // Contains the brightness level for LED 4
#define LED5_BRIGHTNESS  0x0D   // Contains the brightness level for LED 5
#define LED6_BRIGHTNESS  0x0E   // Contains the brightness level for LED 6
#define LED7_BRIGHTNESS  0x0F   // Contains the brightness level for LED 7
#define LED8_BRIGHTNESS  0x10   // Contains the brightness level for LED 8
#define LED9_BRIGHTNESS  0x11   // Contains the brightness level for LED 9
#define LED10_BRIGHTNESS 0x12   // Contains the brightness level for LED 10
#define LED11_BRIGHTNESS 0x13   // Contains the brightness level for LED 11

#define OUT0_COLOR  0x14   // Contains the color value for output 0
#define OUT1_COLOR  0x15   // Contains the color value for output 1
#define OUT2_COLOR  0x16   // Contains the color value for output 2
#define OUT3_COLOR  0x17   // Contains the color value for output 3
#define OUT4_COLOR  0x18   // Contains the color value for output 4
#define OUT5_COLOR  0x19   // Contains the color value for output 5
#define OUT6_COLOR  0x1A   // Contains the color value for output 6
#define OUT7_COLOR  0x1B   // Contains the color value for output 7
#define OUT8_COLOR  0x1C   // Contains the color value for output 8
#define OUT9_COLOR  0x1D   // Contains the color value for output 9
#define OUT10_COLOR 0x1E   // Contains the color value for output 10
#define OUT11_COLOR 0x1F   // Contains the color value for output 11
#define OUT12_COLOR 0x20   // Contains the color value for output 12
#define OUT13_COLOR 0x21   // Contains the color value for output 13
#define OUT14_COLOR 0x22   // Contains the color value for output 14
#define OUT15_COLOR 0x23   // Contains the color value for output 15
#define OUT16_COLOR 0x24   // Contains the color value for output 16
#define OUT17_COLOR 0x25   // Contains the color value for output 17
#define OUT18_COLOR 0x26   // Contains the color value for output 18
#define OUT19_COLOR 0x27   // Contains the color value for output 19
#define OUT20_COLOR 0x28   // Contains the color value for output 20
#define OUT21_COLOR 0x29   // Contains the color value for output 21
#define OUT22_COLOR 0x2A   // Contains the color value for output 22
#define OUT23_COLOR 0x2B   // Contains the color value for output 23
#define OUT24_COLOR 0x2C   // Contains the color value for output 24
#define OUT25_COLOR 0x2D   // Contains the color value for output 25
#define OUT26_COLOR 0x2E   // Contains the color value for output 26
#define OUT27_COLOR 0x2F   // Contains the color value for output 27
#define OUT28_COLOR 0x30   // Contains the color value for output 28
#define OUT29_COLOR 0x31   // Contains the color value for output 29
#define OUT30_COLOR 0x32   // Contains the color value for output 30
#define OUT31_COLOR 0x33   // Contains the color value for output 31
#define OUT32_COLOR 0x34   // Contains the color value for output 32
#define OUT33_COLOR 0x35   // Contains the color value for output 33
#define OUT34_COLOR 0x36   // Contains the color value for output 34
#define OUT35_COLOR 0x37   // Contains the color value for output 35

#define RESET_REGISTERS 0x38   // Resets all the registers to their default values

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
      byte register_address = LED0_BRIGHTNESS + led_number;
      return twi_write(register_address, brightness);
    }

    bool set_led_output_color_by_number(byte led_number, byte color)
    {
      byte register_address = OUT0_COLOR + led_number;
      return twi_write(register_address, color);
    }

    bool set_led_output_color_by_register(byte register_address, byte color)
    {
      return twi_write(register_address, color);
    }

    bool reset_registers()
    {
      return twi_write(RESET_REGISTERS, 0xFF);
    }

    bool enable_chip()
    {
      return twi_write(DEVICE_CONFIG0, 0b01000000);
    }

    bool disable_chip()
    {
      return twi_write(DEVICE_CONFIG0, 0b00000000);
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