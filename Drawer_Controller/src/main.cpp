#include <Arduino.h>
#include <mcp_can.h>
#include <FastLED.h>

#include "robast_can_msgs/can_db.h"
#include "robast_can_msgs/can_helper.h"

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

byte data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
long unsigned int rx_msg_id;
uint8_t rx_msg_dlc = 0;
uint8_t rx_data_buf[8];
char msgString[128];                        // Array to store serial string


/*********************************************************************************************************
  FUNCTIONS
*********************************************************************************************************/


void initialize_voltage_translator(void)
{
  pinMode(OE_TXB0104, OUTPUT);
  digitalWrite(OE_TXB0104, HIGH); // enable voltage level translator
}

void initialize_can_controller(void)
{
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
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

void loop()
{
  
  // can_db.can_messages[CAN_MSG_DRAWER_USER_ACCESS].can_signals[CAN_SIGNAL_DRAWER_ID].data

  // if (digitalRead(SENSOR_DRAWER1_CLOSED_PIN))
  // {
  //   data[0] = 0x00;
  //   Serial.println("SENSOR_DRAWER1_CLOSED_PIN is HIGH");
  // }
  // else
  // {
  //   data[0] = 0x01;
  //   Serial.println("SENSOR_DRAWER1_CLOSED_PIN is LOW");
  // }

  // // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
  // byte sndStat = CAN0.sendMsgBuf(0x100, 0, 8, data);
  // if(sndStat == CAN_OK){
  //   Serial.println("Message Sent Successfully!");
  // } else {
  //   Serial.print("Error Sending Message... CAN Status is: ");
  //   Serial.println(sndStat);
  // }
  // delay(100);


  if(!digitalRead(MCP2515_INT))                         // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rx_msg_id, &rx_msg_dlc, rx_data_buf);

    robast_can_msgs::CanDb can_db = robast_can_msgs::CanDb();
    std::optional<robast_can_msgs::CanMessage> can_message = robast_can_msgs::decode_can_message(rx_msg_id, rx_data_buf, rx_msg_dlc, can_db.can_messages);


    // Serial.println("Received CAN message!");
    
    if((rx_msg_id & 0x80000000) == 0x80000000) // Determine if ID is standard (11 bits) or extended (29 bits)
    {
      Serial.println("Extended ID");
      Serial.printf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rx_msg_id & 0x1FFFFFFF), rx_msg_dlc);
    }     
    else
    {
      Serial.print("Standard ID: ");
      Serial.print(rx_msg_id, BIN);
      Serial.print(" rx_dlc: ");
      Serial.println(uint8_t(rx_msg_dlc), DEC);
      Serial.printf(msgString, "Standard ID: 0x%.3lX  DLC: %1d  Data:", rx_msg_id, rx_msg_dlc);

      if (can_message.has_value())
      {
        if (can_message.value().id == CAN_ID_DRAWER_USER_ACCESS)
        {
          if (can_message.value().can_signals.at(CAN_SIGNAL_OPEN_DRAWER).data == 1) {
            for(int i = 0; i < NUM_LEDS; i++)
            {   
              leds[i] = CRGB::SeaGreen;
            }
          }
          else
          {
            for(int i = 0; i < NUM_LEDS; i++)
            {   
              leds[i] = CRGB::DarkOrange;
            }
          }
          FastLED.setBrightness(10);
          FastLED.show();
        }
      }
    }

    
    
  }
}


