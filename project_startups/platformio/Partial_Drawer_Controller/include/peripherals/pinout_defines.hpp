#ifndef DRAWER_CONTROLLER_PINOUT_DEFINES_HPP
#define DRAWER_CONTROLLER_PINOUT_DEFINES_HPP

/*********************************************************************************************************
  PINOUT DEFINES
*********************************************************************************************************/

namespace peripherals
{
  namespace pinout
  {
    constexpr gpio_num_t TWAI_RX_PIN = GPIO_NUM_4;
    constexpr gpio_num_t TWAI_TX_PIN = GPIO_NUM_5;

    constexpr gpio_num_t SPI_MOSI = GPIO_NUM_23;
    constexpr gpio_num_t SPI_MISO = GPIO_NUM_19;
    constexpr gpio_num_t SPI_CLK = GPIO_NUM_18;
    constexpr gpio_num_t SPI_CS = GPIO_NUM_33;

    constexpr gpio_num_t LED_PIXEL_PIN = GPIO_NUM_32;
  }   // namespace pinout

  namespace i2c
  {
    // TODO: In the hardware design of v1 the pins of SDA and SCL are mixed up unfortunately for the port expander
    // TODO: In order to use the hardware anyway, we need to create two instances of the bus with different pin init
    constexpr gpio_num_t I2C_SDA = GPIO_NUM_21;
    constexpr gpio_num_t I2C_SCL = GPIO_NUM_22;

    constexpr gpio_num_t I2C_SDA_PORT_EXPANDER = GPIO_NUM_22;
    constexpr gpio_num_t I2C_SCL_PORT_EXPANDER = GPIO_NUM_21;
  }   // namespace i2c
}   // namespace peripherals

#endif /* DRAWER_CONTROLLER_PINOUT_DEFINES_HPP */
