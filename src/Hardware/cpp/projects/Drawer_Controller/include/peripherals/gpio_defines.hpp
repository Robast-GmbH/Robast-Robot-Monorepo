#ifndef DRAWER_CONTROLLER_GPIO_DEFINES_HPP
#define DRAWER_CONTROLLER_GPIO_DEFINES_HPP

#include <PCA95x5.h>

#include "gpio/gpio_info.hpp"

namespace drawer_controller
{
  using GpioInfo = gpio::GpioInfo;

  using port_info = std::tuple<uint8_t, PCA95x5::Port::Port>;

  namespace slave_address
  {
    constexpr uint8_t PORT_EXPANDER_1 = 0x20;
  }   // namespace slave_address

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

    constexpr uint8_t LOCK_1_OPEN_CONTROL = 30;
    constexpr uint8_t LOCK_1_CLOSE_CONTROL = 31;
    constexpr uint8_t LOCK_1_SENSE = 32;

    constexpr uint8_t SENSE_INPUT_DRAWER_1_CLOSED = 40;
    constexpr uint8_t PE_0_NINT = 41;

    constexpr uint8_t PE_0_IO0_3 = 50;
    constexpr uint8_t PE_0_IO0_4 = 51;
    constexpr uint8_t PE_0_IO0_5 = 52;
    constexpr uint8_t PE_0_IO0_6 = 53;
    constexpr uint8_t PE_0_IO0_7 = 54;
  }   // namespace pin_id

  const std::unordered_map<uint8_t, GpioInfo> pin_mapping_id_to_gpio_info = {
    {pin_id::STEPPER_1_ENCODER_B, GpioInfo{GPIO_NUM_2, gpio::IS_INPUT}},
    {pin_id::STEPPER_1_ENCODER_A, GpioInfo{GPIO_NUM_26, gpio::IS_INPUT}},
    {pin_id::STEPPER_1_ENCODER_N, GpioInfo{GPIO_NUM_34, gpio::IS_INPUT}},
    {pin_id::SENSE_INPUT_DRAWER_1_CLOSED, GpioInfo{GPIO_NUM_35, gpio::IS_INPUT}},
    {pin_id::PE_0_NINT, GpioInfo{GPIO_NUM_36, gpio::IS_INPUT}},
    {pin_id::LOCK_1_SENSE, GpioInfo{GPIO_NUM_39, gpio::IS_INPUT}}};

  const std::unordered_map<uint8_t, std::shared_ptr<PCA9535>> slave_address_to_port_expander = {
    {slave_address::PORT_EXPANDER_1, std::make_shared<PCA9535>()},
  };

  const std::unordered_map<uint8_t, port_info> pin_mapping_id_to_port = {
    {pin_id::STEPPER_1_ENN_TMC2209, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P00}},
    {pin_id::LOCK_1_OPEN_CONTROL, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P01}},
    {pin_id::LOCK_1_CLOSE_CONTROL, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P02}},
    {pin_id::PE_0_IO0_3, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P03}},
    {pin_id::PE_0_IO0_4, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P04}},
    {pin_id::PE_0_IO0_5, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P05}},
    {pin_id::PE_0_IO0_6, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P06}},
    {pin_id::PE_0_IO0_7, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P07}},
    {pin_id::STEPPER_1_INDEX, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P10}},
    {pin_id::STEPPER_1_STEP, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P12}},
    {pin_id::STEPPER_1_STDBY_TMC2209, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P14}},
    {pin_id::STEPPER_1_DIAG, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P15}},
    {pin_id::STEPPER_1_DIR, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P16}},
    {pin_id::STEPPER_1_SPREAD, {slave_address::PORT_EXPANDER_1, PCA95x5::Port::P17}},
  };

}   // namespace drawer_controller

#endif   // DRAWER_CONTROLLER_GPIO_DEFINES_HPP