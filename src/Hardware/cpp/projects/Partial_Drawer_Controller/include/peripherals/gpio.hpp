#ifndef DRAWER_CONTROLLER_GPIO_HPP
#define DRAWER_CONTROLLER_GPIO_HPP

#include <Arduino.h>
#include <PCA95x5.h>
#include <Wire.h>

#include "interfaces/i_gpio_wrapper.hpp"
#include "peripherals/gpio_info.hpp"
#include "peripherals/pinout_defines.hpp"

namespace drawer_controller
{

  /*********************************************************************************************************
   Slave address defines
  *********************************************************************************************************/

#define SLAVE_ADDRESS_PORT_EXPANDER_1 0x20
#define SLAVE_ADDRESS_PORT_EXPANDER_2 0x21
#define SLAVE_ADDRESS_PORT_EXPANDER_3 0x22

#define SLAVE_ADDRESS_LP5030RJVR           0x33   // individual address, configured by hardware pins
#define SLAVE_ADDRESS_LP5030RJVR_BROADCAST 0x1C   // broadcast to all LP5030RJVRs (up to 4 devices on same I2C bus)

  /*********************************************************************************************************
  These defines are only mapping ID's used within this class
  *********************************************************************************************************/
#define STEPPER_1_ENN_TMC2209_PIN_ID   0
#define STEPPER_1_STDBY_TMC2209_PIN_ID 1
#define STEPPER_1_SPREAD_PIN_ID        2
#define STEPPER_1_DIR_PIN_ID           3
#define STEPPER_1_DIAG_PIN_ID          4
#define STEPPER_1_INDEX_PIN_ID         5
#define STEPPER_1_STEP_PIN_ID          6
#define STEPPER_1_VREF_PIN_ID          7
#define STEPPER_1_ENCODER_A_PIN_ID     8
#define STEPPER_1_ENCODER_B_PIN_ID     9
#define STEPPER_1_ENCODER_N_PIN_ID     10

#define CAN_EN_HIGH_SPEED_MODE_PIN_ID 20
#define ENABLE_ONBOARD_LED_VDD_PIN_ID 21

#define LOCK_1_OPEN_CONTROL_PIN_ID 30
#define LOCK_2_OPEN_CONTROL_PIN_ID 31
#define LOCK_3_OPEN_CONTROL_PIN_ID 32
#define LOCK_4_OPEN_CONTROL_PIN_ID 33
#define LOCK_5_OPEN_CONTROL_PIN_ID 34
#define LOCK_6_OPEN_CONTROL_PIN_ID 35
#define LOCK_7_OPEN_CONTROL_PIN_ID 36
#define LOCK_8_OPEN_CONTROL_PIN_ID 37

#define LOCK_1_CLOSE_CONTROL_PIN_ID 40
#define LOCK_2_CLOSE_CONTROL_PIN_ID 41
#define LOCK_3_CLOSE_CONTROL_PIN_ID 42
#define LOCK_4_CLOSE_CONTROL_PIN_ID 43
#define LOCK_5_CLOSE_CONTROL_PIN_ID 44
#define LOCK_6_CLOSE_CONTROL_PIN_ID 45
#define LOCK_7_CLOSE_CONTROL_PIN_ID 46
#define LOCK_8_CLOSE_CONTROL_PIN_ID 47

#define SENSE_INPUT_LID_1_CLOSED_PIN_ID 50
#define SENSE_INPUT_LID_2_CLOSED_PIN_ID 51
#define SENSE_INPUT_LID_3_CLOSED_PIN_ID 52
#define SENSE_INPUT_LID_4_CLOSED_PIN_ID 53
#define SENSE_INPUT_LID_5_CLOSED_PIN_ID 54
#define SENSE_INPUT_LID_6_CLOSED_PIN_ID 55
#define SENSE_INPUT_LID_7_CLOSED_PIN_ID 56
#define SENSE_INPUT_LID_8_CLOSED_PIN_ID 57

#define SENSE_INPUT_DRAWER_1_CLOSED_PIN_ID 60

#define PE_2_IO0_0_PIN_ID 70
#define PE_2_IO0_3_PIN_ID 71
#define PE_2_IO0_4_PIN_ID 72
#define PE_2_IO1_1_PIN_ID 73
#define PE_2_IO1_6_PIN_ID 74

  using port_info = std::tuple<uint8_t, PCA95x5::Port::Port>;

  class GPIO : public IGpioWrapper
  {
   public:
    GPIO(std::shared_ptr<TwoWire> wire) : _wire{wire}
    {
      for (auto& it : _port_expanders)
      {
        std::shared_ptr<PCA9535> port_expander = it.second;
        uint8_t slave_address = it.first;

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
    bool set_pin_mode(byte pin_mapping_id, bool is_input)
    {
      if (_pin_mapping_id_to_port.find(pin_mapping_id) == _pin_mapping_id_to_port.end())
      {
        if (_pin_mapping_id_to_gpio_info.find(pin_mapping_id) == _pin_mapping_id_to_gpio_info.end())
        {
          Serial.printf("Error! Pin mapping ID %d not found when setting pin mode!\n", pin_mapping_id);
          return false;
        }

        if (_pin_mapping_id_to_gpio_info.at(pin_mapping_id).is_input && !is_input)
        {
          Serial.printf("Error! Trying to set pin mode for pin id %d to input, but it is input only!\n",
                        pin_mapping_id);
          return false;
        }

        pinMode(_pin_mapping_id_to_gpio_info.at(pin_mapping_id).pin_number, is_input ? INPUT : OUTPUT);
        return true;
      }

      port_info port_info = _pin_mapping_id_to_port.at(pin_mapping_id);

      uint8_t port_expander_id = std::get<0>(port_info);
      PCA95x5::Port::Port port_id = std::get<1>(port_info);

      return _port_expanders.at(port_expander_id)
        ->direction(port_id, (is_input == PCA95x5::Direction::IN) ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
    }

    /**
     * Read the digital input on a Input pin.
     *
     * @param pin_mapping_id mapping ID for the input pin to map the required behaviour
     * @param value pointer to the value which will contain the result of the digital read
     * @return if the digital_read was successfull
     */
    bool digital_read(byte pin_mapping_id, byte& value)
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

      value = _port_expanders.at(port_expander_id)->read(port_id);
      return true;
    }

    /**
     * Write the digital output on a output pin.
     *
     * @param pin_mapping_id mapping ID for the output pin to map the required behaviour
     * @param state target state value of the output
     * @return if the digital_write was successfull
     */
    bool digital_write(byte pin_mapping_id, bool state)
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

      return _port_expanders.at(port_expander_id)->write(port_id, state ? PCA95x5::Level::H : PCA95x5::Level::L);
    }

    bool get_gpio_output_pin_mode()
    {
      return PCA95x5::Direction::OUT;
    }

    bool get_gpio_input_pin_mode()
    {
      return PCA95x5::Direction::IN;
    }

   private:
    std::shared_ptr<TwoWire> _wire;

    const std::unordered_map<uint8_t, GpioInfo> _pin_mapping_id_to_gpio_info = {
      {SENSE_INPUT_LID_8_CLOSED_PIN_ID, GpioInfo{GPIO_NUM_0, IS_INPUT}}};

    const std::unordered_map<uint8_t, std::shared_ptr<PCA9535>> _port_expanders = {
      {SLAVE_ADDRESS_PORT_EXPANDER_1, std::make_shared<PCA9535>()},
      {SLAVE_ADDRESS_PORT_EXPANDER_2, std::make_shared<PCA9535>()},
      {SLAVE_ADDRESS_PORT_EXPANDER_3, std::make_shared<PCA9535>()},
    };

    const std::unordered_map<uint8_t, port_info> _pin_mapping_id_to_port = {
      {STEPPER_1_STDBY_TMC2209_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P00}},
      {SENSE_INPUT_LID_3_CLOSED_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P01}},
      {LOCK_2_OPEN_CONTROL_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P02}},
      {LOCK_2_CLOSE_CONTROL_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P03}},
      {SENSE_INPUT_LID_2_CLOSED_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P04}},
      {LOCK_1_OPEN_CONTROL_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P05}},
      {LOCK_1_CLOSE_CONTROL_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P06}},
      {SENSE_INPUT_LID_1_CLOSED_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P07}},
      {STEPPER_1_ENN_TMC2209_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P10}},
      {LOCK_3_OPEN_CONTROL_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P11}},
      {LOCK_3_CLOSE_CONTROL_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P12}},
      {STEPPER_1_DIR_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P13}},
      {STEPPER_1_SPREAD_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P14}},
      {STEPPER_1_DIAG_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P15}},
      {STEPPER_1_STEP_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P16}},
      {STEPPER_1_INDEX_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P17}},
      {LOCK_8_CLOSE_CONTROL_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_2, PCA95x5::Port::P06}},
      {LOCK_8_OPEN_CONTROL_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_2, PCA95x5::Port::P07}},
      {CAN_EN_HIGH_SPEED_MODE_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_2, PCA95x5::Port::P10}},
      {LOCK_7_CLOSE_CONTROL_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_2, PCA95x5::Port::P11}},
      {LOCK_7_OPEN_CONTROL_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_2, PCA95x5::Port::P12}},
      {SENSE_INPUT_LID_7_CLOSED_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_2, PCA95x5::Port::P15}},
      {LOCK_6_OPEN_CONTROL_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_2, PCA95x5::Port::P16}},
      {LOCK_6_CLOSE_CONTROL_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_2, PCA95x5::Port::P17}},
      {PE_2_IO0_0_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_3, PCA95x5::Port::P00}},
      {SENSE_INPUT_DRAWER_1_CLOSED_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_3, PCA95x5::Port::P01}},
      {LOCK_4_CLOSE_CONTROL_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_3, PCA95x5::Port::P02}},
      {PE_2_IO0_3_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_3, PCA95x5::Port::P03}},
      {PE_2_IO0_4_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_3, PCA95x5::Port::P04}},
      {SENSE_INPUT_LID_5_CLOSED_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_3, PCA95x5::Port::P05}},
      {LOCK_4_OPEN_CONTROL_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_3, PCA95x5::Port::P06}},
      {ENABLE_ONBOARD_LED_VDD_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_3, PCA95x5::Port::P07}},
      {SENSE_INPUT_LID_6_CLOSED_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_3, PCA95x5::Port::P10}},
      {PE_2_IO1_1_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_3, PCA95x5::Port::P11}},
      {STEPPER_1_ENCODER_N_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_3, PCA95x5::Port::P12}},
      {SENSE_INPUT_LID_4_CLOSED_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_3, PCA95x5::Port::P14}},
      {LOCK_5_CLOSE_CONTROL_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_3, PCA95x5::Port::P15}},
      {PE_2_IO1_6_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_3, PCA95x5::Port::P16}},
      {LOCK_5_OPEN_CONTROL_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_3, PCA95x5::Port::P17}},
    };
  };

}   // namespace drawer_controller
#endif   // DRAWER_CONTROLLER_GPIO_HPP
