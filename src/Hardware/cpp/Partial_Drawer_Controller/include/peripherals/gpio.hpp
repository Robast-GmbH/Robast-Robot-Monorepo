#ifndef DRAWER_CONTROLLER_GPIO_HPP
#define DRAWER_CONTROLLER_GPIO_HPP

#include <Arduino.h>
#include <PCA95x5.h>
#include <Wire.h>

#include "interfaces/i_gpio_wrapper.hpp"
#include "peripherals/pinout_defines.hpp"

namespace drawer_controller
{

  /*********************************************************************************************************
   Slave address defines
  *********************************************************************************************************/

#define SLAVE_ADDRESS_PORT_EXPANDER_1 0x20
#define SLAVE_ADDRESS_PORT_EXPANDER_2 0x21
#define SLAVE_ADDRESS_PORT_EXPANDER_3 0x22

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

  class GPIO : public IGpioWrapper
  {
   public:
    GPIO()
    {
      Wire.begin(I2C_SDA, I2C_SCL);
      port_expander_0_.attach(Wire, SLAVE_ADDRESS_PORT_EXPANDER_1);
      port_expander_1_.attach(Wire, SLAVE_ADDRESS_PORT_EXPANDER_2);
      port_expander_2_.attach(Wire, SLAVE_ADDRESS_PORT_EXPANDER_3);

      port_expander_0_.polarity(PCA95x5::Polarity::ORIGINAL_ALL);
      port_expander_1_.polarity(PCA95x5::Polarity::ORIGINAL_ALL);
      port_expander_2_.polarity(PCA95x5::Polarity::ORIGINAL_ALL);
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
      switch (pin_mapping_id)
      {
        case STEPPER_1_STDBY_TMC2209_PIN_ID:
          return port_expander_0_.direction(PCA95x5::Port::P00,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case SENSE_INPUT_LID_3_CLOSED_PIN_ID:
          return port_expander_0_.direction(PCA95x5::Port::P01,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case LOCK_2_OPEN_CONTROL_PIN_ID:
          return port_expander_0_.direction(PCA95x5::Port::P02,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case LOCK_2_CLOSE_CONTROL_PIN_ID:
          return port_expander_0_.direction(PCA95x5::Port::P03,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case SENSE_INPUT_LID_2_CLOSED_PIN_ID:
          return port_expander_0_.direction(PCA95x5::Port::P04,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case LOCK_1_OPEN_CONTROL_PIN_ID:
          return port_expander_0_.direction(PCA95x5::Port::P05,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case LOCK_1_CLOSE_CONTROL_PIN_ID:
          return port_expander_0_.direction(PCA95x5::Port::P06,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case SENSE_INPUT_LID_1_CLOSED_PIN_ID:
          return port_expander_0_.direction(PCA95x5::Port::P07,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case STEPPER_1_ENN_TMC2209_PIN_ID:
          return port_expander_0_.direction(PCA95x5::Port::P10,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case LOCK_3_OPEN_CONTROL_PIN_ID:
          return port_expander_0_.direction(PCA95x5::Port::P11,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case LOCK_3_CLOSE_CONTROL_PIN_ID:
          return port_expander_0_.direction(PCA95x5::Port::P12,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case STEPPER_1_DIR_PIN_ID:
          return port_expander_0_.direction(PCA95x5::Port::P13,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case STEPPER_1_SPREAD_PIN_ID:
          return port_expander_0_.direction(PCA95x5::Port::P14,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case STEPPER_1_DIAG_PIN_ID:
          return port_expander_0_.direction(PCA95x5::Port::P15,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case STEPPER_1_STEP_PIN_ID:
          return port_expander_0_.direction(PCA95x5::Port::P16,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case STEPPER_1_INDEX_PIN_ID:
          return port_expander_0_.direction(PCA95x5::Port::P17,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case LOCK_8_CLOSE_CONTROL_PIN_ID:
          return port_expander_1_.direction(PCA95x5::Port::P06,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case LOCK_8_OPEN_CONTROL_PIN_ID:
          return port_expander_1_.direction(PCA95x5::Port::P07,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case CAN_EN_HIGH_SPEED_MODE_PIN_ID:
          return port_expander_1_.direction(PCA95x5::Port::P10,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case LOCK_7_CLOSE_CONTROL_PIN_ID:
          return port_expander_1_.direction(PCA95x5::Port::P11,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case LOCK_7_OPEN_CONTROL_PIN_ID:
          return port_expander_1_.direction(PCA95x5::Port::P12,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case SENSE_INPUT_LID_7_CLOSED_PIN_ID:
          return port_expander_1_.direction(PCA95x5::Port::P15,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case LOCK_6_OPEN_CONTROL_PIN_ID:
          return port_expander_1_.direction(PCA95x5::Port::P16,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case LOCK_6_CLOSE_CONTROL_PIN_ID:
          return port_expander_1_.direction(PCA95x5::Port::P17,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case PE_2_IO0_0_PIN_ID:
          return port_expander_2_.direction(PCA95x5::Port::P00,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case SENSE_INPUT_DRAWER_1_CLOSED_PIN_ID:
          return port_expander_2_.direction(PCA95x5::Port::P01,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case LOCK_4_CLOSE_CONTROL_PIN_ID:
          return port_expander_2_.direction(PCA95x5::Port::P02,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case PE_2_IO0_3_PIN_ID:
          return port_expander_2_.direction(PCA95x5::Port::P03,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case PE_2_IO0_4_PIN_ID:
          return port_expander_2_.direction(PCA95x5::Port::P04,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case SENSE_INPUT_LID_5_CLOSED_PIN_ID:
          return port_expander_2_.direction(PCA95x5::Port::P05,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case LOCK_4_OPEN_CONTROL_PIN_ID:
          return port_expander_2_.direction(PCA95x5::Port::P06,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case ENABLE_ONBOARD_LED_VDD_PIN_ID:
          return port_expander_2_.direction(PCA95x5::Port::P07,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case SENSE_INPUT_LID_6_CLOSED_PIN_ID:
          return port_expander_2_.direction(PCA95x5::Port::P10,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case PE_2_IO1_1_PIN_ID:
          return port_expander_2_.direction(PCA95x5::Port::P11,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case STEPPER_1_ENCODER_N_PIN_ID:
          return port_expander_2_.direction(PCA95x5::Port::P12,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case SENSE_INPUT_LID_4_CLOSED_PIN_ID:
          return port_expander_2_.direction(PCA95x5::Port::P14,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case LOCK_5_CLOSE_CONTROL_PIN_ID:
          return port_expander_2_.direction(PCA95x5::Port::P15,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case PE_2_IO1_6_PIN_ID:
          return port_expander_2_.direction(PCA95x5::Port::P16,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        case LOCK_5_OPEN_CONTROL_PIN_ID:
          return port_expander_2_.direction(PCA95x5::Port::P17,
                                            is_input ? PCA95x5::Direction::IN : PCA95x5::Direction::OUT);
          break;

        default:
          break;
      }

      return false;
    }

    /**
     * Read the digital input on a Input pin.
     *
     * @param pin_mapping_id mapping ID for the input pin to map the required behaviour
     * @param value pointer to the value which will contain the result of the digital read
     * @return if the digital_read was successfull
     */
    bool digital_read(byte pin_mapping_id, byte &value)
    {
      switch (pin_mapping_id)
      {
        case SENSE_INPUT_LOCK_1_PIN_ID:
          value = PCA9554_REGISTER_3;
          return port_expander_2_.digital_read(value);
          break;

        case SENSE_INPUT_LOCK_2_PIN_ID:
          value = PCA9554_REGISTER_0;
          return port_expander_2_.digital_read(value);
          break;

        // TODO@Jacob: The pins for SENSE_INPUT_DRAWER_1_CLOSED_PIN_ID and SENSE_INPUT_DRAWER_2_CLOSED_PIN_ID
        // TODO@Jacob: will be remapped again in the next interation probably
        case SENSE_INPUT_DRAWER_1_CLOSED_PIN_ID:
          value = PCA9554_REGISTER_2;
          return port_expander_2_.digital_read(value);
          break;

        case SENSE_INPUT_DRAWER_2_CLOSED_PIN_ID:
          value = PCA9554_REGISTER_1;
          return port_expander_2_.digital_read(value);
          break;

        case STEPPER_1_DIAG_PIN_ID:
          value = PCA9554_REGISTER_4;
          return port_expander_2_.digital_read(value);
          break;

        case STEPPER_2_DIAG_PIN_ID:
          value = PCA9554_REGISTER_5;
          return port_expander_2_.digital_read(value);
          break;

        case MCP2515_RX0BF_PIN_ID:
          value = PCA9554_REGISTER_6;
          return port_expander_2_.digital_read(value);
          break;

        case MCP2515_RX1BF_PIN_ID:
          value = PCA9554_REGISTER_7;
          return port_expander_2_.digital_read(value);
          break;

        default:
          break;
      }
      return false;
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
      switch (pin_mapping_id)
      {
        case STEPPER_1_ENN_TMC2209_PIN_ID:
          return port_expander_0_.digital_write(PCA9554_REGISTER_2, state);
          break;

        case STEPPER_2_EN_TMC2209_PIN_ID:
          return port_expander_0_.digital_write(PCA9554_REGISTER_0, state);
          break;

        case STEPPER_1_STDBY_TMC2209_PIN_ID:
          return port_expander_0_.digital_write(PCA9554_REGISTER_3, state);
          break;

        case STEPPER_2_STDBY_TMC2209_PIN_ID:
          return port_expander_0_.digital_write(PCA9554_REGISTER_1, state);
          break;

        case STEPPER_1_SPREAD_PIN_ID:
          return port_expander_0_.digital_write(PCA9554_REGISTER_4, state);
          break;

        case STEPPER_2_SPREAD_PIN_ID:
          return port_expander_0_.digital_write(PCA9554_REGISTER_6, state);
          break;

        case STEPPER_1_DIR_PIN_ID:
          return port_expander_1_.digital_write(PCA9554_REGISTER_7, state);
          break;

        case STEPPER_2_DIR_PIN_ID:
          return port_expander_0_.digital_write(PCA9554_REGISTER_5, state);
          break;

        case OE_TXB0104_PIN_ID:
          return port_expander_0_.digital_write(PCA9554_REGISTER_7, state);
          break;

        case LOCK_1_OPEN_CONROL_PIN_ID:
          return port_expander_1_.digital_write(PCA9554_REGISTER_2, state);
          break;

        case LOCK_2_OPEN_CONROL_PIN_ID:
          return port_expander_1_.digital_write(PCA9554_REGISTER_0, state);
          break;

        case LOCK_1_CLOSE_CONROL_PIN_ID:
          return port_expander_1_.digital_write(PCA9554_REGISTER_3, state);
          break;

        case LOCK_2_CLOSE_CONROL_PIN_ID:
          return port_expander_1_.digital_write(PCA9554_REGISTER_1, state);
          break;

        default:
          break;
      }
      return false;
    }

    bool get_gpio_output_pin_mode()
    {
      return PCA9554_OUTPUT;
    }

    bool get_gpio_input_pin_mode()
    {
      return PCA9554_INPUT;
    }

   private:
    PCA9535 port_expander_0_;
    PCA9535 port_expander_1_;
    PCA9535 port_expander_2_;
  };

}   // namespace drawer_controller
#endif   // DRAWER_CONTROLLER_GPIO_HPP
