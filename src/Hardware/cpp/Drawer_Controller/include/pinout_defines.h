#ifndef DRAWER_CONTROLLER_PINOUT_DEFINES_HPP
#define DRAWER_CONTROLLER_PINOUT_DEFINES_HPP

/*********************************************************************************************************
  PINOUT DEFINES
*********************************************************************************************************/

#define PWR_OPEN_LOCK1_PIN GPIO_NUM_22
#define PWR_CLOSE_LOCK1_PIN GPIO_NUM_21
#define SENSOR_LOCK1_PIN GPIO_NUM_36
#define SENSOR_DRAWER1_CLOSED_PIN GPIO_NUM_39

#define PWR_OPEN_LOCK2_PIN GPIO_NUM_4
#define PWR_CLOSE_LOCK2_PIN GPIO_NUM_15
#define SENSOR_LOCK2_PIN GPIO_NUM_34
#define SENSOR_DRAWER2_CLOSED_PIN GPIO_NUM_35

#define OE_TXB0104 GPIO_NUM_32
#define MCP2515_INT GPIO_NUM_25
#define MCP2515_RX0BF GPIO_NUM_26
#define MCP2515_RX1BF GPIO_NUM_27

#define SPI_MOSI GPIO_NUM_23
#define SPI_MISO GPIO_NUM_19
#define SPI_CLK GPIO_NUM_18
#define SPI_CS GPIO_NUM_5

#define LED_PIXEL_PIN GPIO_NUM_13

#endif /* DRAWER_CONTROLLER_PINOUT_DEFINES_HPP */