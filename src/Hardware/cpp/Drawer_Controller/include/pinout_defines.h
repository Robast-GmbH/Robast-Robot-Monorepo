#ifndef DRAWER_CONTROLLER_PINOUT_DEFINES_HPP
#define DRAWER_CONTROLLER_PINOUT_DEFINES_HPP

/*********************************************************************************************************
  PINOUT DEFINES
*********************************************************************************************************/

#define MCP2515_INT GPIO_NUM_4

#define SPI_MOSI GPIO_NUM_23
#define SPI_MISO GPIO_NUM_19
#define SPI_CLK  GPIO_NUM_18
#define SPI_CS   GPIO_NUM_5

#define I2C_SCL GPIO_NUM_22
#define I2C_SDA GPIO_NUM_21

#define STEPPER_ENABLE_PIN GPIO_NUM_13
#define ENCODER_A_PIN      GPIO_NUM_4
#define ENCODER_B_PIN      GPIO_NUM_15
#define STEPPER_DIAG_PIN   GPIO_NUM_27

#define LED_PIXEL_PIN GPIO_NUM_32

#endif /* DRAWER_CONTROLLER_PINOUT_DEFINES_HPP */
