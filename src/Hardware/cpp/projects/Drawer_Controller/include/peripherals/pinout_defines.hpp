#ifndef DRAWER_CONTROLLER_PINOUT_DEFINES_HPP
#define DRAWER_CONTROLLER_PINOUT_DEFINES_HPP

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
    constexpr gpio_num_t I2C_SDA = GPIO_NUM_22;
    constexpr gpio_num_t I2C_SCL = GPIO_NUM_21;
  }   // namespace i2c

}   // namespace peripherals

#endif /* DRAWER_CONTROLLER_PINOUT_DEFINES_HPP */
