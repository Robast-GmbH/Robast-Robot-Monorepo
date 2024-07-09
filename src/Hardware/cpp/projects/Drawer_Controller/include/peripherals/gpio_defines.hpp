#ifndef DRAWER_CONTROLLER_GPIO_DEFINES_HPP
#define DRAWER_CONTROLLER_GPIO_DEFINES_HPP

#include <PCA95x5.h>

#include "gpio/gpio_info.hpp"

namespace drawer_controller
{

  using port_info = std::tuple<uint8_t, PCA95x5::Port::Port>;

  /*********************************************************************************************************
  Slave address defines
 *********************************************************************************************************/

#define SLAVE_ADDRESS_PORT_EXPANDER_1 0x20

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

#define LOCK_1_OPEN_CONTROL_PIN_ID  30
#define LOCK_1_CLOSE_CONTROL_PIN_ID 31
#define LOCK_1_SENSE_PIN_ID         32

#define SENSE_INPUT_DRAWER_1_CLOSED_PIN_ID 40
#define PE_0_NINT                          41

#define PE_0_IO0_3_PIN_ID 50
#define PE_0_IO0_4_PIN_ID 51
#define PE_0_IO0_5_PIN_ID 52
#define PE_0_IO0_6_PIN_ID 53
#define PE_0_IO0_7_PIN_ID 54

  const std::unordered_map<uint8_t, drawer_controller::GpioInfo> pin_mapping_id_to_gpio_info = {
    {STEPPER_1_ENCODER_B_PIN_ID, drawer_controller::GpioInfo{GPIO_NUM_2, IS_INPUT}},
    {STEPPER_1_ENCODER_A_PIN_ID, drawer_controller::GpioInfo{GPIO_NUM_26, IS_INPUT}},
    {STEPPER_1_ENCODER_N_PIN_ID, drawer_controller::GpioInfo{GPIO_NUM_34, IS_INPUT}},
    {SENSE_INPUT_DRAWER_1_CLOSED_PIN_ID, drawer_controller::GpioInfo{GPIO_NUM_35, IS_INPUT}},
    {PE_0_NINT, drawer_controller::GpioInfo{GPIO_NUM_36, IS_INPUT}},
    {LOCK_1_SENSE_PIN_ID, drawer_controller::GpioInfo{GPIO_NUM_39, IS_INPUT}}};

  const std::unordered_map<uint8_t, std::shared_ptr<PCA9535>> port_expanders = {
    {SLAVE_ADDRESS_PORT_EXPANDER_1, std::make_shared<PCA9535>()},
  };

  const std::unordered_map<uint8_t, port_info> pin_mapping_id_to_port = {
    {STEPPER_1_ENN_TMC2209_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P00}},
    {LOCK_1_OPEN_CONTROL_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P01}},
    {LOCK_1_CLOSE_CONTROL_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P02}},
    {PE_0_IO0_3_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P03}},
    {PE_0_IO0_4_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P04}},
    {PE_0_IO0_5_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P05}},
    {PE_0_IO0_6_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P06}},
    {PE_0_IO0_7_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P07}},
    {STEPPER_1_INDEX_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P10}},
    {STEPPER_1_STEP_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P12}},
    {STEPPER_1_STDBY_TMC2209_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P14}},
    {STEPPER_1_DIAG_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P15}},
    {STEPPER_1_DIR_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P16}},
    {STEPPER_1_SPREAD_PIN_ID, {SLAVE_ADDRESS_PORT_EXPANDER_1, PCA95x5::Port::P17}},
  };

}   // namespace drawer_controller

#endif   // DRAWER_CONTROLLER_GPIO_DEFINES_HPP