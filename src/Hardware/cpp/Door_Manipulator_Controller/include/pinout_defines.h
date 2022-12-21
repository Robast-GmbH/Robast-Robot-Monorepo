#ifndef PINOUT_DEFINES_HPP_
#define PINOUT_DEFINES_HPP_

/*********************************************************************************************************
  PINOUT DEFINES
*********************************************************************************************************/


#define MOTOR_1_START_PIN GPIO_NUM_22 // PWR_OPEN_LOCK1_PIN
#define MOTOR_1_INPUT_1_PIN GPIO_NUM_33
#define MOTOR_1_INPUT_2_PIN GPIO_NUM_12
#define MOTOR_1_INPUT_3_PIN GPIO_NUM_14
//#define MOTOR1_INPUT4

#define MOTOR_2_START_PIN GPIO_NUM_21 // PWR_CLOSE_LOCK1_PIN
#define MOTOR_2_INPUT_1_PIN GPIO_NUM_17
#define MOTOR_2_INPUT_2_PIN GPIO_NUM_16
#define MOTOR_2_INPUT_3_PIN GPIO_NUM_2
//#define MOTOR2_INPUT4

#define MOTOR_3_START_PIN GPIO_NUM_4 // PWR_OPEN_LOCK2_PIN
#define MOTOR_3_INPUT_1_PIN GPIO_NUM_15
#define MOTOR_3_INPUT_2_PIN GPIO_NUM_13
#define MOTOR_3_INPUT_3_PIN GPIO_NUM_0
//#define MOTOR3_INPUT4


#define SENSOR_LOCK1_PIN GPIO_NUM_36 // only input
#define SENSOR_DRAWER1_CLOSED_PIN GPIO_NUM_39 // only input

#define SENSOR_LOCK2_PIN GPIO_NUM_34 // only input
#define SENSOR_DRAWER2_CLOSED_PIN GPIO_NUM_35 // only input

#define OE_TXB0104 GPIO_NUM_32
#define MCP2515_INT GPIO_NUM_25
#define MCP2515_RX0BF GPIO_NUM_26
#define MCP2515_RX1BF GPIO_NUM_27

#define SPI_MOSI GPIO_NUM_23
#define SPI_MISO GPIO_NUM_19
#define SPI_CLK GPIO_NUM_18
#define SPI_CS GPIO_NUM_5


#endif /* PINOUT_DEFINES_HPP_ */