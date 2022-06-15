#include <Arduino.h>
#include <mcp_can.h>
#include <FastLED.h>

/*********************************************************************************************************
  GLOBAL VARIABLES AND CONSTANTS
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

#define NUM_LEDS 19 // number of LEDs for LED strip

CRGBArray<NUM_LEDS> leds;

MCP_CAN CAN0(SPI_CS);


/*********************************************************************************************************
  FUNCTIONS
*********************************************************************************************************/

void initialize_can_controller(void)
{
  if(CAN0.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) == CAN_OK)
  {
    Serial.println("MCP2515 Initialized Successfully!");
  }
  else 
  {
    Serial.println("Error Initializing MCP2515...");
  } 

  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted

  pinMode(MCP2515_INT, INPUT);  // Configuring pin for /INT input
}

void initialize_voltage_translator(void)
{
  pinMode(OE_TXB0104, OUTPUT);
  digitalWrite(OE_TXB0104, HIGH); // enable voltage level translator
}

void initialize_locks(void)
{
  pinMode(PWR_OPEN_LOCK1_PIN, OUTPUT);
  pinMode(PWR_CLOSE_LOCK1_PIN, OUTPUT);
  pinMode(SENSOR_LOCK1_PIN, INPUT);
  pinMode(SENSOR_DRAWER1_CLOSED_PIN, INPUT);

  pinMode(PWR_OPEN_LOCK2_PIN, OUTPUT);
  pinMode(PWR_CLOSE_LOCK2_PIN, OUTPUT);
  pinMode(SENSOR_LOCK2_PIN, INPUT);
  pinMode(SENSOR_DRAWER2_CLOSED_PIN, INPUT);

  digitalWrite(PWR_OPEN_LOCK1_PIN, LOW);
  digitalWrite(PWR_CLOSE_LOCK1_PIN, LOW);

  digitalWrite(PWR_OPEN_LOCK2_PIN, LOW);
  digitalWrite(PWR_CLOSE_LOCK2_PIN, LOW);
}

void initialize_LED_strip(void)
{
 FastLED.addLeds<NEOPIXEL,LED_PIXEL_PIN>(leds, NUM_LEDS);
}

/*********************************************************************************************************
  SETUP
*********************************************************************************************************/

void setup()
{
  Serial.begin(115200);

  initialize_voltage_translator();  

  initialize_can_controller();

  initialize_locks();

  initialize_LED_strip();
}


/*********************************************************************************************************
  LOOP
*********************************************************************************************************/

byte data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};

void loop()
{
  if (digitalRead(SENSOR_DRAWER1_CLOSED_PIN))
  {
    data[0] = 0x00;
    Serial.println("SENSOR_DRAWER1_CLOSED_PIN is HIGH");
  }
  else
  {
    data[0] = 0x01;
    Serial.println("SENSOR_DRAWER1_CLOSED_PIN is LOW");
  }

  // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
  byte sndStat = CAN0.sendMsgBuf(0x100, 0, 8, data);
  if(sndStat == CAN_OK){
    Serial.println("Message Sent Successfully!");
  } else {
    Serial.print("Error Sending Message... CAN Status is: ");
    Serial.println(sndStat);
  }
  delay(2000);
}


