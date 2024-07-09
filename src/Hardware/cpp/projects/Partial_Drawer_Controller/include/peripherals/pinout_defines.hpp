#ifndef DRAWER_CONTROLLER_PINOUT_DEFINES_HPP
#define DRAWER_CONTROLLER_PINOUT_DEFINES_HPP

/*********************************************************************************************************
  PINOUT DEFINES
*********************************************************************************************************/

#define TWAI_RX_PIN GPIO_NUM_4
#define TWAI_TX_PIN GPIO_NUM_5

#define SPI_MOSI GPIO_NUM_23
#define SPI_MISO GPIO_NUM_19
#define SPI_CLK  GPIO_NUM_18
#define SPI_CS   GPIO_NUM_33

#define LED_PIXEL_PIN GPIO_NUM_32

/*********************************************************************************************************
  I2C
*********************************************************************************************************/

// TODO: In the hardware design of v1 the pins of SDA and SCL are mixed up unfortunately for the port expander
// TODO: In order to use the hardware anyway, we need to create two instances of the bus with different pin init

#define I2C_SDA GPIO_NUM_21
#define I2C_SCL GPIO_NUM_22

#define I2C_SDA_PORT_EXPANDER GPIO_NUM_22
#define I2C_SCL_PORT_EXPANDER GPIO_NUM_21

#endif /* DRAWER_CONTROLLER_PINOUT_DEFINES_HPP */
