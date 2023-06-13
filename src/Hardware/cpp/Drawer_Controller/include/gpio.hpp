#if !defined(DRAWER_CONTROLLER_GPIO_HPP)
#define DRAWER_CONTROLLER_GPIO_HPP

#include <Arduino.h>
#include <Wire.h>

#include "i_gpio_wrapper.hpp"
#include "pinout_defines.h"
#include "port_expander_pca9554.hpp"

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
#define DRAWER_1_EN_TMC2209_PIN_ID    0
#define DRAWER_2_EN_TMC2209_PIN_ID    1
#define DRAWER_1_STDBY_TMC2209_PIN_ID 2
#define DRAWER_2_STDBY_TMC2209_PIN_ID 3
#define DRAWER_1_SPREAD_PIN_ID        4
#define DRAWER_2_SPREAD_PIN_ID        5
#define DRAWER_1_DIR_PIN_ID           6
#define DRAWER_2_DIR_PIN_ID           7
#define DRAWER_1_DIAG_PIN_ID          8
#define DRAWER_2_DIAG_PIN_ID          9
#define DRAWER_1_INDEX_PIN_ID         10
#define DRAWER_2_INDEX_PIN_ID         11
#define DRAWER_1_STEP_PIN_ID          12
#define DRAWER_2_STEP_PIN_ID          13
#define DRAWER_1_ENCODER_A_PIN_ID     14
#define DRAWER_1_ENCODER_B_PIN_ID     15
#define DRAWER_1_ENCODER_N_PIN_ID     16
#define DRAWER_2_ENCODER_A_PIN_ID     17
#define DRAWER_2_ENCODER_B_PIN_ID     18
#define DRAWER_2_ENCODER_N_PIN_ID     19

#define LOCK_1_OPEN_CONROL_PIN_ID  30
#define LOCK_2_OPEN_CONROL_PIN_ID  31
#define LOCK_1_CLOSE_CONROL_PIN_ID 32
#define LOCK_2_CLOSE_CONROL_PIN_ID 33

#define SENSE_INPUT_DRAWER_1_CLOSED_PIN_ID 40
#define SENSE_INPUT_DRAWER_2_CLOSED_PIN_ID 41
#define SENSE_INPUT_LOCK_1_PIN_ID          42
#define SENSE_INPUT_LOCK_2_PIN_ID          43

#define MCP2515_RX0BF_PIN_ID 50
#define MCP2515_RX1BF_PIN_ID 51

#define OE_TXB0104_PIN_ID 60

  class GPIO : public IGpioWrapper
  {
   public:
    GPIO()
    {
      Wire.begin(I2C_SDA, I2C_SCL);
    }

    // void initialize_pins()
    // {
    //   port_expander_0_.set_pin_mode(PCA9554_REGISTER_0, OUTPUT);   // Drawer_2_~EN_TMC2209
    //   port_expander_0_.set_pin_mode(PCA9554_REGISTER_1, OUTPUT);   // Drawer_2_STDBY_TMC2209
    //   port_expander_0_.set_pin_mode(PCA9554_REGISTER_2, OUTPUT);   // Drawer_1_~EN_TMC2209
    //   port_expander_0_.set_pin_mode(PCA9554_REGISTER_3, OUTPUT);   // Drawer_1_STDBY_TMC2209
    //   port_expander_0_.set_pin_mode(PCA9554_REGISTER_4, OUTPUT);   // Drawer_1_SPREAD
    //   port_expander_0_.set_pin_mode(PCA9554_REGISTER_5, OUTPUT);   // Drawer_2_DIR
    //   port_expander_0_.set_pin_mode(PCA9554_REGISTER_6, OUTPUT);   // Drawer_2_SPREAD
    //   port_expander_0_.set_pin_mode(PCA9554_REGISTER_7, OUTPUT);   // OE_TXB0104

    //   port_expander_1_.set_pin_mode(PCA9554_REGISTER_0, OUTPUT);   // Lock_2_Open_Control
    //   port_expander_1_.set_pin_mode(PCA9554_REGISTER_1, OUTPUT);   // Lock_2_Close_Control
    //   port_expander_1_.set_pin_mode(PCA9554_REGISTER_2, OUTPUT);   // Lock_1_Open_Control
    //   port_expander_1_.set_pin_mode(PCA9554_REGISTER_3, OUTPUT);   // Lock_1_Close_Control
    //   // port_expander_1_.pin_mode(PCA9554_REGISTER_4, OUTPUT);   // Not in use
    //   // port_expander_1_.pin_mode(PCA9554_REGISTER_5, OUTPUT);   // Not in use
    //   // port_expander_1_.pin_mode(PCA9554_REGISTER_6, OUTPUT);   // Not in use
    //   port_expander_1_.set_pin_mode(PCA9554_REGISTER_7, OUTPUT);   // Drawer_1_DIR

    //   port_expander_2_.set_pin_mode(PCA9554_REGISTER_0, INPUT);    // Sense_Input_Lock_2
    //   port_expander_2_.set_pin_mode(PCA9554_REGISTER_1, INPUT);    // Sense_Input_Drawer_Closed_1
    //   port_expander_2_.set_pin_mode(PCA9554_REGISTER_2, INPUT);    // Sense_Input_Drawer_Closed_2
    //   port_expander_2_.set_pin_mode(PCA9554_REGISTER_3, INPUT);    // Sense_Input_Lock_1
    //   port_expander_2_.set_pin_mode(PCA9554_REGISTER_4, INPUT);    // Drawer_1_DIAG
    //   port_expander_2_.set_pin_mode(PCA9554_REGISTER_5, INPUT);    // Drawer_2_DIAG
    //   port_expander_2_.set_pin_mode(PCA9554_REGISTER_6, INPUT);    // MCP2515_RX0BF
    //   port_expander_2_.set_pin_mode(PCA9554_REGISTER_7, INPUT);    // MCP2515_RX1BF
    // }

    /**
     * Set the pin mode for the pin mapped to the given pin_mapping_id
     *
     * @param pin_mapping_id mapping ID for the pin whose mode should be set
     * @param state defines whether the pin should be an input or an output pin
     * @return if the digital_read was successfull
     */
    bool set_pin_mode(byte pin_mapping_id, bool state)
    {
      switch (pin_mapping_id)
      {
        case DRAWER_2_EN_TMC2209_PIN_ID:
          return port_expander_0_.set_pin_mode(PCA9554_REGISTER_0, state);
          break;

        case DRAWER_2_STDBY_TMC2209_PIN_ID:
          return port_expander_0_.set_pin_mode(PCA9554_REGISTER_1, state);
          break;

        case DRAWER_1_EN_TMC2209_PIN_ID:
          return port_expander_0_.set_pin_mode(PCA9554_REGISTER_2, state);
          break;

        case DRAWER_1_STDBY_TMC2209_PIN_ID:
          return port_expander_0_.set_pin_mode(PCA9554_REGISTER_3, state);
          break;

        case DRAWER_1_SPREAD_PIN_ID:
          return port_expander_0_.set_pin_mode(PCA9554_REGISTER_4, state);
          break;

        case DRAWER_2_DIR_PIN_ID:
          return port_expander_0_.set_pin_mode(PCA9554_REGISTER_5, state);
          break;

        case DRAWER_2_SPREAD_PIN_ID:
          return port_expander_0_.set_pin_mode(PCA9554_REGISTER_6, state);
          break;

        case OE_TXB0104_PIN_ID:
          return port_expander_0_.set_pin_mode(PCA9554_REGISTER_7, state);
          break;

        case LOCK_2_OPEN_CONROL_PIN_ID:
          return port_expander_1_.set_pin_mode(PCA9554_REGISTER_0, state);
          break;

        case LOCK_2_CLOSE_CONROL_PIN_ID:
          return port_expander_1_.set_pin_mode(PCA9554_REGISTER_1, state);
          break;

        case LOCK_1_OPEN_CONROL_PIN_ID:
          return port_expander_1_.set_pin_mode(PCA9554_REGISTER_2, state);
          break;

        case LOCK_1_CLOSE_CONROL_PIN_ID:
          return port_expander_1_.set_pin_mode(PCA9554_REGISTER_3, state);
          break;

        case DRAWER_1_DIR_PIN_ID:
          return port_expander_1_.set_pin_mode(PCA9554_REGISTER_7, state);
          break;

        case SENSE_INPUT_LOCK_2_PIN_ID:
          return port_expander_2_.set_pin_mode(PCA9554_REGISTER_0, state);
          break;

        case SENSE_INPUT_DRAWER_1_CLOSED_PIN_ID:
          return port_expander_2_.set_pin_mode(PCA9554_REGISTER_1, state);
          break;

        case SENSE_INPUT_DRAWER_2_CLOSED_PIN_ID:
          return port_expander_2_.set_pin_mode(PCA9554_REGISTER_2, state);
          break;

        case SENSE_INPUT_LOCK_1_PIN_ID:
          return port_expander_2_.set_pin_mode(PCA9554_REGISTER_3, state);
          break;

        case DRAWER_1_DIAG_PIN_ID:
          return port_expander_2_.set_pin_mode(PCA9554_REGISTER_4, state);
          break;

        case DRAWER_2_DIAG_PIN_ID:
          return port_expander_2_.set_pin_mode(PCA9554_REGISTER_5, state);
          break;

        case MCP2515_RX0BF_PIN_ID:
          return port_expander_2_.set_pin_mode(PCA9554_REGISTER_6, state);
          break;

        case MCP2515_RX1BF_PIN_ID:
          return port_expander_2_.set_pin_mode(PCA9554_REGISTER_7, state);
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
          value = 3;
          return port_expander_2_.digital_read(value);
          break;

        case SENSE_INPUT_LOCK_2_PIN_ID:
          value = 0;
          return port_expander_2_.digital_read(value);
          break;

        case SENSE_INPUT_DRAWER_1_CLOSED_PIN_ID:
          value = 1;
          return port_expander_2_.digital_read(value);
          break;

        case SENSE_INPUT_DRAWER_2_CLOSED_PIN_ID:
          value = 2;
          return port_expander_2_.digital_read(value);
          break;

        case DRAWER_1_DIAG_PIN_ID:
          value = 4;
          return port_expander_2_.digital_read(value);
          break;

        case DRAWER_2_DIAG_PIN_ID:
          value = 5;
          return port_expander_2_.digital_read(value);
          break;

        case MCP2515_RX0BF_PIN_ID:
          value = 6;
          return port_expander_2_.digital_read(value);
          break;

        case MCP2515_RX1BF_PIN_ID:
          value = 7;
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
        case DRAWER_1_EN_TMC2209_PIN_ID:
          return port_expander_0_.digital_write(PCA9554_REGISTER_2, state);
          break;

        case DRAWER_2_EN_TMC2209_PIN_ID:
          return port_expander_0_.digital_write(PCA9554_REGISTER_0, state);
          break;

        case DRAWER_1_STDBY_TMC2209_PIN_ID:
          return port_expander_0_.digital_write(PCA9554_REGISTER_3, state);
          break;

        case DRAWER_2_STDBY_TMC2209_PIN_ID:
          return port_expander_0_.digital_write(PCA9554_REGISTER_1, state);
          break;

        case DRAWER_1_SPREAD_PIN_ID:
          return port_expander_0_.digital_write(PCA9554_REGISTER_4, state);
          break;

        case DRAWER_2_SPREAD_PIN_ID:
          return port_expander_0_.digital_write(PCA9554_REGISTER_6, state);
          break;

        case DRAWER_1_DIR_PIN_ID:
          return port_expander_1_.digital_write(PCA9554_REGISTER_7, state);
          break;

        case DRAWER_2_DIR_PIN_ID:
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

   private:
    port_expander::PortExpanderPCA9554 port_expander_0_ =
        port_expander::PortExpanderPCA9554(SLAVE_ADDRESS_PORT_EXPANDER_1);
    port_expander::PortExpanderPCA9554 port_expander_1_ =
        port_expander::PortExpanderPCA9554(SLAVE_ADDRESS_PORT_EXPANDER_2);
    port_expander::PortExpanderPCA9554 port_expander_2_ =
        port_expander::PortExpanderPCA9554(SLAVE_ADDRESS_PORT_EXPANDER_3);
  };

}   // namespace drawer_controller
#endif   // DRAWER_CONTROLLER_GPIO_HPP
