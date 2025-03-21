#ifndef LED_ONBOARD_LED_DRIVER_HPP
#define LED_ONBOARD_LED_DRIVER_HPP

#include <Arduino.h>
#include <Wire.h>

#include <memory>

#include "led/LP503x.hpp"

namespace led
{
  namespace slave_address
  {
    constexpr uint8_t LP5030RJVR = 0x33;             // individual address, configured by hardware pins
    constexpr uint8_t LP5030RJVR_BROADCAST = 0x1C;   // broadcast to all LP5030RJVRs (up to 4 devices on same I2C bus)
  }   // namespace slave_address

  namespace tray_id
  {
    constexpr uint8_t TRAY_1 = 1;
    constexpr uint8_t TRAY_2 = 2;
    constexpr uint8_t TRAY_3 = 3;
    constexpr uint8_t TRAY_4 = 4;
    constexpr uint8_t TRAY_5 = 5;
    constexpr uint8_t TRAY_6 = 6;
    constexpr uint8_t TRAY_7 = 7;
    constexpr uint8_t TRAY_8 = 8;
  }   // namespace tray_id

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
    explicit OnboardLedDriver(std::shared_ptr<TwoWire> wire)
    {
      _lp5030rjvr = std::make_shared<lp503x::LP503x>(slave_address::LP5030RJVR, wire);
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
      if (tray < tray_id::TRAY_1 || tray > tray_id::TRAY_8)
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
      {tray_id::TRAY_1,
       {lp503x::register_address::OUT_16_COLOR,
        lp503x::register_address::OUT_15_COLOR,
        lp503x::register_address::OUT_17_COLOR}},
      {tray_id::TRAY_2,
       {lp503x::register_address::OUT_19_COLOR,
        lp503x::register_address::OUT_20_COLOR,
        lp503x::register_address::OUT_21_COLOR}},
      {tray_id::TRAY_3,
       {lp503x::register_address::OUT_18_COLOR,
        lp503x::register_address::OUT_22_COLOR,
        lp503x::register_address::OUT_23_COLOR}},
      {tray_id::TRAY_4,
       {lp503x::register_address::OUT_14_COLOR,
        lp503x::register_address::OUT_6_COLOR,
        lp503x::register_address::OUT_4_COLOR}},
      {tray_id::TRAY_5,
       {lp503x::register_address::OUT_3_COLOR,
        lp503x::register_address::OUT_2_COLOR,
        lp503x::register_address::OUT_1_COLOR}},
      {tray_id::TRAY_6,
       {lp503x::register_address::OUT_5_COLOR,
        lp503x::register_address::OUT_7_COLOR,
        lp503x::register_address::OUT_0_COLOR}},
      {tray_id::TRAY_7,
       {lp503x::register_address::OUT_12_COLOR,
        lp503x::register_address::OUT_9_COLOR,
        lp503x::register_address::OUT_8_COLOR}},
      {tray_id::TRAY_8,
       {lp503x::register_address::OUT_11_COLOR,
        lp503x::register_address::OUT_10_COLOR,
        lp503x::register_address::OUT_13_COLOR}},
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

}   // namespace led
#endif   // LED_ONBOARD_LED_DRIVER_HPP
