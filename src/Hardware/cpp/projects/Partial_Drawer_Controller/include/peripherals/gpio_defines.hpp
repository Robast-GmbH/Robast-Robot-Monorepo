#ifndef PARTIAL_DRAWER_CONTROLLER_GPIO_DEFINES_HPP
#define PARTIAL_DRAWER_CONTROLLER_GPIO_DEFINES_HPP

#include <PCA95x5.h>

#include "gpio/gpio_info.hpp"

namespace partial_drawer_controller
{
  using GpioInfo = gpio::GpioInfo;

  using port_info = std::tuple<uint8_t, PCA95x5::Port::Port>;

  /*********************************************************************************************************
  Slave address defines
 *********************************************************************************************************/

  namespace slave_address
  {
    constexpr uint8_t PORT_EXPANDER_1 = 0x20;
    constexpr uint8_t PORT_EXPANDER_2 = 0x21;
    constexpr uint8_t PORT_EXPANDER_3 = 0x22;

    constexpr uint8_t LP5030RJVR = 0x33;             // individual address, configured by hardware pins
    constexpr uint8_t LP5030RJVR_BROADCAST = 0x1C;   // broadcast to all LP5030RJVRs (up to 4 devices on same I2C bus)
  }   // namespace slave_address

  /*********************************************************************************************************
  These defines are only mapping ID's used within this class
  *********************************************************************************************************/

  namespace pin_id
  {
    constexpr uint8_t STEPPER_1_ENN_TMC2209 = 0;
    constexpr uint8_t STEPPER_1_STDBY_TMC2209 = 1;
    constexpr uint8_t STEPPER_1_SPREAD = 2;
    constexpr uint8_t STEPPER_1_DIR = 3;
    constexpr uint8_t STEPPER_1_DIAG = 4;
    constexpr uint8_t STEPPER_1_INDEX = 5;
    constexpr uint8_t STEPPER_1_STEP = 6;
    constexpr uint8_t STEPPER_1_VREF = 7;
    constexpr uint8_t STEPPER_1_ENCODER_A = 8;
    constexpr uint8_t STEPPER_1_ENCODER_B = 9;
    constexpr uint8_t STEPPER_1_ENCODER_N = 10;

    constexpr uint8_t CAN_EN_HIGH_SPEED_MODE = 20;
    constexpr uint8_t ENABLE_ONBOARD_LED_VDD = 21;

    constexpr uint8_t LOCK_1_OPEN_CONTROL = 30;
    constexpr uint8_t LOCK_2_OPEN_CONTROL = 31;
    constexpr uint8_t LOCK_3_OPEN_CONTROL = 32;
    constexpr uint8_t LOCK_4_OPEN_CONTROL = 33;
    constexpr uint8_t LOCK_5_OPEN_CONTROL = 34;
    constexpr uint8_t LOCK_6_OPEN_CONTROL = 35;
    constexpr uint8_t LOCK_7_OPEN_CONTROL = 36;
    constexpr uint8_t LOCK_8_OPEN_CONTROL = 37;

    constexpr uint8_t LOCK_1_CLOSE_CONTROL = 40;
    constexpr uint8_t LOCK_2_CLOSE_CONTROL = 41;
    constexpr uint8_t LOCK_3_CLOSE_CONTROL = 42;
    constexpr uint8_t LOCK_4_CLOSE_CONTROL = 43;
    constexpr uint8_t LOCK_5_CLOSE_CONTROL = 44;
    constexpr uint8_t LOCK_6_CLOSE_CONTROL = 45;
    constexpr uint8_t LOCK_7_CLOSE_CONTROL = 46;
    constexpr uint8_t LOCK_8_CLOSE_CONTROL = 47;

    constexpr uint8_t SENSE_INPUT_LID_1_CLOSED = 50;
    constexpr uint8_t SENSE_INPUT_LID_2_CLOSED = 51;
    constexpr uint8_t SENSE_INPUT_LID_3_CLOSED = 52;
    constexpr uint8_t SENSE_INPUT_LID_4_CLOSED = 53;
    constexpr uint8_t SENSE_INPUT_LID_5_CLOSED = 54;
    constexpr uint8_t SENSE_INPUT_LID_6_CLOSED = 55;
    constexpr uint8_t SENSE_INPUT_LID_7_CLOSED = 56;
    constexpr uint8_t SENSE_INPUT_LID_8_CLOSED = 57;

    constexpr uint8_t SENSE_INPUT_DRAWER_1_CLOSED = 60;

    constexpr uint8_t PE_2_IO0_0 = 70;
    constexpr uint8_t PE_2_IO0_3 = 71;
    constexpr uint8_t PE_2_IO0_4 = 72;
    constexpr uint8_t PE_2_IO1_1 = 73;
    constexpr uint8_t PE_2_IO1_6 = 74;

    constexpr uint8_t PE_0_NINTERRUPT = 80;
    constexpr uint8_t PE_1_NINTERRUPT = 81;
    constexpr uint8_t PE_2_NINTERRUPT = 82;
  }   // namespace pin_id

  const std::unordered_map<uint8_t, GpioInfo> pin_mapping_id_to_gpio_info = {
    {pin_id::STEPPER_1_ENCODER_B, GpioInfo{GPIO_NUM_2, gpio::IS_INPUT}},
    {pin_id::STEPPER_1_VREF, GpioInfo{GPIO_NUM_25, gpio::IS_OUTPUT}},
    {pin_id::STEPPER_1_ENCODER_A, GpioInfo{GPIO_NUM_26, gpio::IS_INPUT}},
    {pin_id::PE_1_NINTERRUPT, GpioInfo{GPIO_NUM_34, gpio::IS_INPUT}},
    {pin_id::SENSE_INPUT_LID_8_CLOSED, GpioInfo{GPIO_NUM_35, gpio::IS_INPUT}},
    {pin_id::PE_0_NINTERRUPT, GpioInfo{GPIO_NUM_36, gpio::IS_INPUT}},
    {pin_id::PE_2_NINTERRUPT, GpioInfo{GPIO_NUM_39, gpio::IS_INPUT}}};

  const std::unordered_map<uint8_t, std::shared_ptr<PCA9535>> slave_address_to_port_expander = {
    {slave_address::PORT_EXPANDER_1, std::make_shared<PCA9535>()},
    {slave_address::PORT_EXPANDER_2, std::make_shared<PCA9535>()},
    {slave_address::PORT_EXPANDER_3, std::make_shared<PCA9535>()},
  };

  const std::unordered_map<uint8_t, port_info> pin_mapping_id_to_port = {
    {pin_id::STEPPER_1_STDBY_TMC2209, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P00}},
    {pin_id::SENSE_INPUT_LID_3_CLOSED, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P01}},
    {pin_id::LOCK_2_OPEN_CONTROL, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P02}},
    {pin_id::LOCK_2_CLOSE_CONTROL, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P03}},
    {pin_id::SENSE_INPUT_LID_2_CLOSED, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P04}},
    {pin_id::LOCK_1_OPEN_CONTROL, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P05}},
    {pin_id::LOCK_1_CLOSE_CONTROL, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P06}},
    {pin_id::SENSE_INPUT_LID_1_CLOSED, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P07}},
    {pin_id::STEPPER_1_ENN_TMC2209, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P10}},
    {pin_id::LOCK_3_OPEN_CONTROL, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P11}},
    {pin_id::LOCK_3_CLOSE_CONTROL, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P12}},
    {pin_id::STEPPER_1_DIR, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P13}},
    {pin_id::STEPPER_1_SPREAD, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P14}},
    {pin_id::STEPPER_1_DIAG, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P15}},
    {pin_id::STEPPER_1_STEP, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P16}},
    {pin_id::STEPPER_1_INDEX, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P17}},
    {pin_id::LOCK_8_CLOSE_CONTROL, {slave_address::PORT_EXPANDER_2, PCA95x5::Port::P06}},
    {pin_id::LOCK_8_OPEN_CONTROL, {slave_address::PORT_EXPANDER_2, PCA95x5::Port::P07}},
    {pin_id::CAN_EN_HIGH_SPEED_MODE, {slave_address::PORT_EXPANDER_2, PCA95x5::Port::P10}},
    {pin_id::LOCK_7_CLOSE_CONTROL, {slave_address::PORT_EXPANDER_2, PCA95x5::Port::P11}},
    {pin_id::LOCK_7_OPEN_CONTROL, {slave_address::PORT_EXPANDER_2, PCA95x5::Port::P12}},
    {pin_id::SENSE_INPUT_LID_7_CLOSED, {slave_address::PORT_EXPANDER_2, PCA95x5::Port::P15}},
    {pin_id::LOCK_6_OPEN_CONTROL, {slave_address::PORT_EXPANDER_2, PCA95x5::Port::P16}},
    {pin_id::LOCK_6_CLOSE_CONTROL, {slave_address::PORT_EXPANDER_2, PCA95x5::Port::P17}},
    {pin_id::PE_2_IO0_0, {slave_address::PORT_EXPANDER_3, PCA95x5::Port::P00}},
    {pin_id::SENSE_INPUT_DRAWER_1_CLOSED, {slave_address::PORT_EXPANDER_3, PCA95x5::Port::P01}},
    {pin_id::LOCK_4_CLOSE_CONTROL, {slave_address::PORT_EXPANDER_3, PCA95x5::Port::P02}},
    {pin_id::PE_2_IO0_3, {slave_address::PORT_EXPANDER_3, PCA95x5::Port::P03}},
    {pin_id::PE_2_IO0_4, {slave_address::PORT_EXPANDER_3, PCA95x5::Port::P04}},
    {pin_id::SENSE_INPUT_LID_5_CLOSED, {slave_address::PORT_EXPANDER_3, PCA95x5::Port::P05}},
    {pin_id::LOCK_4_OPEN_CONTROL, {slave_address::PORT_EXPANDER_3, PCA95x5::Port::P06}},
    {pin_id::ENABLE_ONBOARD_LED_VDD, {slave_address::PORT_EXPANDER_3, PCA95x5::Port::P07}},
    {pin_id::SENSE_INPUT_LID_6_CLOSED, {slave_address::PORT_EXPANDER_3, PCA95x5::Port::P10}},
    {pin_id::PE_2_IO1_1, {slave_address::PORT_EXPANDER_3, PCA95x5::Port::P11}},
    {pin_id::STEPPER_1_ENCODER_N, {slave_address::PORT_EXPANDER_3, PCA95x5::Port::P12}},
    {pin_id::SENSE_INPUT_LID_4_CLOSED, {slave_address::PORT_EXPANDER_3, PCA95x5::Port::P14}},
    {pin_id::LOCK_5_CLOSE_CONTROL, {slave_address::PORT_EXPANDER_3, PCA95x5::Port::P15}},
    {pin_id::PE_2_IO1_6, {slave_address::PORT_EXPANDER_3, PCA95x5::Port::P16}},
    {pin_id::LOCK_5_OPEN_CONTROL, {slave_address::PORT_EXPANDER_3, PCA95x5::Port::P17}},
  };

}   // namespace partial_drawer_controller

#endif   // PARTIAL_DRAWER_CONTROLLER_GPIO_DEFINES_HPP