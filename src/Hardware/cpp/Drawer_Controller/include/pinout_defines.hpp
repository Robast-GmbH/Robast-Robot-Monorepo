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

#define DRAWER_1_STEP_PIN      GPIO_NUM_14
#define DRAWER_1_INDEX_PIN     GPIO_NUM_15
#define DRAWER_1_ENCODER_A_PIN GPIO_NUM_33
#define DRAWER_1_ENCODER_B_PIN GPIO_NUM_13
#define DRAWER_1_ENCODER_N_PIN GPIO_NUM_25

#define DRAWER_2_STEP_PIN      GPIO_NUM_12
#define DRAWER_2_INDEX_PIN     GPIO_NUM_2
#define DRAWER_2_ENCODER_A_PIN GPIO_NUM_35
#define DRAWER_2_ENCODER_B_PIN GPIO_NUM_14
#define DRAWER_2_ENCODER_N_PIN GPIO_NUM_36

#define LED_PIXEL_PIN GPIO_NUM_32

#endif /* DRAWER_CONTROLLER_PINOUT_DEFINES_HPP */
