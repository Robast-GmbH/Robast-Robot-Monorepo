#ifndef PARTIAL_DRAWER_CONTROLLER_LED_DRIVER_HPP
#define PARTIAL_DRAWER_CONTROLLER_LED_DRIVER_HPP

#include <Arduino.h>
#include <Wire.h>

#include <memory>

#include "led/LP503x.hpp"
#include "peripherals/pinout_defines.hpp"

namespace partial_drawer_controller
{

  /*********************************************************************************************************
   Slave address defines
  *********************************************************************************************************/

#define SLAVE_ADDRESS_LP5030RJVR           0x33   // individual address, configured by hardware pins
#define SLAVE_ADDRESS_LP5030RJVR_BROADCAST 0x1C   // broadcast to all LP5030RJVRs (up to 4 devices on same I2C bus)

  /*********************************************************************************************************
   * Trays
   *********************************************************************************************************/

#define TRAY_1 1
#define TRAY_2 2
#define TRAY_3 3
#define TRAY_4 4
#define TRAY_5 5
#define TRAY_6 6
#define TRAY_7 7
#define TRAY_8 8

  // One tray has 3 rows of leds with 8 leds each and each row has its own port
  struct tray_led_register_config
  {
    uint8_t register_row_1;
    uint8_t register_row_2;
    uint8_t register_row_3;
  };

  class OnboardLedDriver
  {
   public:
    OnboardLedDriver(std::shared_ptr<TwoWire> wire)
    {
      _lp5030rjvr = std::make_shared<lp503x::LP503x>(SLAVE_ADDRESS_LP5030RJVR, wire);
    }

    void reset_registers()
    {
      _lp5030rjvr->reset_registers();
    }

    void enable_chip()
    {
      _lp5030rjvr->enable_chip();
    }

    void disable_chip()
    {
      _lp5030rjvr->disable_chip();
    }

    void read_register(byte register_address, byte& data_read)
    {
      _lp5030rjvr->read_register(register_address, data_read);
    }

    void initialize(std::function<void()> set_enable_pin_high)
    {
      set_enable_pin_high();
      reset_registers();
      enable_chip();
    }

    void set_tray_led_brightness(uint8_t tray, uint8_t led_row, uint8_t brightness)
    {
      if (tray < TRAY_1 || tray > TRAY_8)
      {
        Serial.printf("Warning! Trying to set led brightness for invalid tray %d!\n", tray);
        return;
      }

      const tray_led_register_config& led_config = _tray_to_led_config.at(tray);

      switch (led_row)
      {
        case 1:
          set_led_brightness_by_register(led_config.register_row_1, brightness);
          break;
        case 2:
          set_led_brightness_by_register(led_config.register_row_2, brightness);
          break;
        case 3:
          set_led_brightness_by_register(led_config.register_row_3, brightness);
          break;
        default:
          Serial.printf("Warning! Trying to set led brightness for tray %d with invalid row %d!\n", tray, led_row);
          break;
      }
    }

   private:
    std::shared_ptr<lp503x::LP503x> _lp5030rjvr;

    const std::unordered_map<uint8_t, tray_led_register_config> _tray_to_led_config = {
      {TRAY_1, {OUT16_COLOR, OUT15_COLOR, OUT17_COLOR}},
      {TRAY_2, {OUT19_COLOR, OUT20_COLOR, OUT21_COLOR}},
      {TRAY_3, {OUT18_COLOR, OUT22_COLOR, OUT23_COLOR}},
      {TRAY_4, {OUT14_COLOR, OUT6_COLOR, OUT4_COLOR}},
      {TRAY_5, {OUT3_COLOR, OUT2_COLOR, OUT1_COLOR}},
      {TRAY_6, {OUT5_COLOR, OUT7_COLOR, OUT0_COLOR}},
      {TRAY_7, {OUT12_COLOR, OUT9_COLOR, OUT8_COLOR}},
      {TRAY_8, {OUT11_COLOR, OUT10_COLOR, OUT13_COLOR}},
    };

    void set_led_output_color_by_number(byte led_number, byte color)
    {
      _lp5030rjvr->set_led_output_color_by_number(led_number, color);
    }

    void set_led_brightness_by_register(byte register_address, byte brightness)
    {
      // For now I did not understand why setting the color register instead of the brightness register is resulting
      // in changing the brightness, but it is working
      _lp5030rjvr->set_led_output_color_by_register(register_address, brightness);
    }
  };

}   // namespace partial_drawer_controller
#endif   // PARTIAL_DRAWER_CONTROLLER_LED_DRIVER_HPP
