#ifndef PERIPHERALS_GPIO_DEFINES_V3_HPP
#define PERIPHERALS_GPIO_DEFINES_V3_HPP

#include <memory>
#include <peripherals/port_expander_pca9554.hpp>

#include "gpio/gpio_info.hpp"

namespace gpio_defines
{
  using GpioInfo = gpio::GpioInfo;

  using slave_address_by_register = std::tuple<uint8_t, uint8_t>;

  namespace slave_address
  {
    constexpr uint8_t PORT_EXPANDER_1 = 0x20;
    constexpr uint8_t PORT_EXPANDER_2 = 0x21;
    constexpr uint8_t PORT_EXPANDER_3 = 0x22;
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
    constexpr uint8_t STEPPER_1_ENCODER_A = 7;
    constexpr uint8_t STEPPER_1_ENCODER_B = 8;
    constexpr uint8_t STEPPER_1_ENCODER_N = 9;

    constexpr uint8_t STEPPER_2_ENN_TMC2209 = 10;
    constexpr uint8_t STEPPER_2_STDBY_TMC2209 = 11;
    constexpr uint8_t STEPPER_2_SPREAD = 12;
    constexpr uint8_t STEPPER_2_DIR = 13;
    constexpr uint8_t STEPPER_2_DIAG = 14;
    constexpr uint8_t STEPPER_2_INDEX = 15;
    constexpr uint8_t STEPPER_2_STEP = 16;
    constexpr uint8_t STEPPER_2_ENCODER_A = 17;
    constexpr uint8_t STEPPER_2_ENCODER_B = 18;
    constexpr uint8_t STEPPER_2_ENCODER_N = 19;

    constexpr uint8_t LOCK_1_OPEN_CONTROL = 30;
    constexpr uint8_t LOCK_1_CLOSE_CONTROL = 31;
    constexpr uint8_t LOCK_1_SENSE = 32;
    constexpr uint8_t LOCK_2_OPEN_CONTROL = 33;
    constexpr uint8_t LOCK_2_CLOSE_CONTROL = 34;
    constexpr uint8_t LOCK_2_SENSE = 35;

    constexpr uint8_t SENSE_INPUT_DRAWER_1_CLOSED = 40;
    constexpr uint8_t SENSE_INPUT_DRAWER_2_CLOSED = 41;

    constexpr uint8_t MCP2515_RX0BF = 50;
    constexpr uint8_t MCP2515_RX1BF = 51;

    constexpr uint8_t OE_TXB0104 = 60;

  }   // namespace pin_id

  const std::unordered_map<uint8_t, GpioInfo> pin_mapping_id_to_gpio_info = {
    {pin_id::STEPPER_1_STEP, GpioInfo{GPIO_NUM_14, gpio::IS_INPUT}},
    {pin_id::STEPPER_1_INDEX, GpioInfo{GPIO_NUM_15, gpio::IS_INPUT}},
    {pin_id::STEPPER_1_ENCODER_A, GpioInfo{GPIO_NUM_33, gpio::IS_INPUT}},
    {pin_id::STEPPER_1_ENCODER_B, GpioInfo{GPIO_NUM_13, gpio::IS_INPUT}},
    {pin_id::STEPPER_1_ENCODER_N, GpioInfo{GPIO_NUM_25, gpio::IS_INPUT}},
    {pin_id::STEPPER_2_STEP, GpioInfo{GPIO_NUM_12, gpio::IS_INPUT}},
    {pin_id::STEPPER_2_INDEX, GpioInfo{GPIO_NUM_2, gpio::IS_INPUT}},
    {pin_id::STEPPER_2_ENCODER_A, GpioInfo{GPIO_NUM_35, gpio::IS_INPUT}},
    {pin_id::STEPPER_2_ENCODER_B, GpioInfo{GPIO_NUM_14, gpio::IS_INPUT}},
    {pin_id::STEPPER_2_ENCODER_N, GpioInfo{GPIO_NUM_36, gpio::IS_INPUT}},
  };

  const std::unordered_map<uint8_t, std::shared_ptr<port_expander::PCA9554>> slave_address_to_port_expander = {
    {slave_address::PORT_EXPANDER_1, std::make_shared<port_expander::PCA9554>()},
    {slave_address::PORT_EXPANDER_2, std::make_shared<port_expander::PCA9554>()},
    {slave_address::PORT_EXPANDER_3, std::make_shared<port_expander::PCA9554>()}};

  const std::unordered_map<uint8_t, slave_address_by_register> pin_mapping_id_to_slave_address_by_register = {
    {pin_id::STEPPER_2_ENN_TMC2209, {slave_address::PORT_EXPANDER_1, port_expander::pca9554::REGISTER_0}},
    {pin_id::STEPPER_2_STDBY_TMC2209, {slave_address::PORT_EXPANDER_1, port_expander::pca9554::REGISTER_1}},
    {pin_id::STEPPER_1_ENN_TMC2209, {slave_address::PORT_EXPANDER_1, port_expander::pca9554::REGISTER_2}},
    {pin_id::STEPPER_1_STDBY_TMC2209, {slave_address::PORT_EXPANDER_1, port_expander::pca9554::REGISTER_3}},
    {pin_id::STEPPER_1_SPREAD, {slave_address::PORT_EXPANDER_1, port_expander::pca9554::REGISTER_4}},
    {pin_id::STEPPER_2_DIR, {slave_address::PORT_EXPANDER_1, port_expander::pca9554::REGISTER_5}},
    {pin_id::STEPPER_2_SPREAD, {slave_address::PORT_EXPANDER_1, port_expander::pca9554::REGISTER_6}},
    {pin_id::OE_TXB0104, {slave_address::PORT_EXPANDER_1, port_expander::pca9554::REGISTER_7}},
    {pin_id::LOCK_2_OPEN_CONTROL, {slave_address::PORT_EXPANDER_2, port_expander::pca9554::REGISTER_0}},
    {pin_id::LOCK_2_CLOSE_CONTROL, {slave_address::PORT_EXPANDER_2, port_expander::pca9554::REGISTER_1}},
    {pin_id::LOCK_1_OPEN_CONTROL, {slave_address::PORT_EXPANDER_2, port_expander::pca9554::REGISTER_2}},
    {pin_id::LOCK_1_CLOSE_CONTROL, {slave_address::PORT_EXPANDER_2, port_expander::pca9554::REGISTER_3}},
    {pin_id::STEPPER_1_DIR, {slave_address::PORT_EXPANDER_2, port_expander::pca9554::REGISTER_7}},
    {pin_id::LOCK_2_SENSE, {slave_address::PORT_EXPANDER_3, port_expander::pca9554::REGISTER_0}},
    {pin_id::SENSE_INPUT_DRAWER_2_CLOSED, {slave_address::PORT_EXPANDER_3, port_expander::pca9554::REGISTER_1}},
    {pin_id::SENSE_INPUT_DRAWER_1_CLOSED, {slave_address::PORT_EXPANDER_3, port_expander::pca9554::REGISTER_2}},
    {pin_id::LOCK_1_SENSE, {slave_address::PORT_EXPANDER_3, port_expander::pca9554::REGISTER_3}},
    {pin_id::STEPPER_1_DIAG, {slave_address::PORT_EXPANDER_3, port_expander::pca9554::REGISTER_4}},
    {pin_id::STEPPER_2_DIAG, {slave_address::PORT_EXPANDER_3, port_expander::pca9554::REGISTER_5}},
    {pin_id::MCP2515_RX0BF, {slave_address::PORT_EXPANDER_3, port_expander::pca9554::REGISTER_6}},
    {pin_id::MCP2515_RX1BF, {slave_address::PORT_EXPANDER_3, port_expander::pca9554::REGISTER_7}}};

}   // namespace gpio_defines

#endif   // PERIPHERALS_GPIO_DEFINES_V3_HPP