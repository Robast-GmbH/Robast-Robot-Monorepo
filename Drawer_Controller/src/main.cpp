#include <Arduino.h>
#include <mcp_can.h>
#include <FastLED.h>

#include "robast_can_msgs/can_db.h"
#include "robast_can_msgs/can_helper.h"

/*********************************************************************************************************
  GLOBAL VARIABLES AND CONSTANTS
*********************************************************************************************************/

#define DRAWER_ID 1 //TODO: Every Drawer needs to have his own id

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

robast_can_msgs::CanDb can_db = robast_can_msgs::CanDb();

long unsigned int rx_msg_id;
uint8_t rx_msg_dlc = 0;
uint8_t rx_data_buf[8];

unsigned long previous_millis = 0;
const unsigned long interval_drawer_feedback_in_ms = 1000;

float moving_average_sensor_lock1_pin = 0;
float moving_average_drawer1_closed_pin = 0;
float moving_average_sensor_lock2_pin = 0;
float moving_average_drawer2_closed_pin = 0;


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

void handle_locks(robast_can_msgs::CanMessage can_message)
{
  if (can_message.can_signals.at(CAN_SIGNAL_OPEN_LOCK_1).data == CAN_DATA_OPEN_LOCK)
  {
    digitalWrite(PWR_OPEN_LOCK1_PIN, HIGH);
  }
  if (can_message.can_signals.at(CAN_SIGNAL_OPEN_LOCK_1).data == CAN_DATA_CLOSE_LOCK)
  {
    digitalWrite(PWR_OPEN_LOCK1_PIN, LOW);
  }

  if (can_message.can_signals.at(CAN_SIGNAL_OPEN_LOCK_2).data == CAN_DATA_OPEN_LOCK)
  {
    digitalWrite(PWR_OPEN_LOCK2_PIN, HIGH);
  }
  if (can_message.can_signals.at(CAN_SIGNAL_OPEN_LOCK_2).data == CAN_DATA_CLOSE_LOCK)
  {
    digitalWrite(PWR_OPEN_LOCK2_PIN, LOW);
  }
}

void handle_LED_strip(robast_can_msgs::CanMessage can_message)
{
  uint8_t red = can_message.can_signals.at(CAN_SIGNAL_LED_RED).data;
  uint8_t green = can_message.can_signals.at(CAN_SIGNAL_LED_GREEN).data;
  uint8_t blue = can_message.can_signals.at(CAN_SIGNAL_LED_BLUE).data;

  for(int i = 0; i < NUM_LEDS; i++)
  {   
    leds[i] = CRGB(red, green, blue);
  }
  FastLED.setBrightness(10);
  FastLED.show();
}

void debug_prints(robast_can_msgs::CanMessage can_message)
{
  Serial.print("Standard ID: ");
  Serial.print(rx_msg_id, HEX);
  Serial.print(" rx_dlc: ");
  Serial.print(uint8_t(rx_msg_dlc), DEC);
  Serial.print(" DRAWER ID: ");
  Serial.print(can_message.can_signals.at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).data, HEX);
  Serial.print(" CAN_SIGNAL_OPEN_LOCK_1: ");
  Serial.print(can_message.can_signals.at(CAN_SIGNAL_OPEN_LOCK_1).data, BIN);
  Serial.print(" CAN_SIGNAL_OPEN_LOCK_2: ");
  Serial.print(can_message.can_signals.at(CAN_SIGNAL_OPEN_LOCK_2).data, BIN);
  Serial.print(" LED RED: ");
  Serial.print(can_message.can_signals.at(CAN_SIGNAL_LED_RED).data, HEX);
  Serial.print(" LED GREEN: ");
  Serial.print(can_message.can_signals.at(CAN_SIGNAL_LED_GREEN).data, HEX);
  Serial.print(" LED BLUE: ");
  Serial.println(can_message.can_signals.at(CAN_SIGNAL_LED_BLUE).data, HEX);
}

void handle_CAN_msg(robast_can_msgs::CanMessage can_message)
{
  if (can_message.id == CAN_ID_DRAWER_USER_ACCESS)
  {
    handle_locks(can_message);

    handle_LED_strip(can_message);

    debug_prints(can_message);
  }
}

void handle_reading_sensors(void)
{
  // Tracking the moving average for the sensor pins helps to debounce them a little bit
  moving_average_sensor_lock1_pin = 0.2 * digitalRead(SENSOR_LOCK1_PIN) + 0.8 * moving_average_sensor_lock1_pin;
  moving_average_drawer1_closed_pin = 0.2 * digitalRead(SENSOR_DRAWER1_CLOSED_PIN) + 0.8 * moving_average_drawer1_closed_pin;
  moving_average_sensor_lock2_pin = 0.2 * digitalRead(SENSOR_LOCK2_PIN) + 0.8 * moving_average_sensor_lock2_pin;
  moving_average_drawer2_closed_pin = 0.2 * digitalRead(SENSOR_DRAWER2_CLOSED_PIN) + 0.8 * moving_average_drawer2_closed_pin;
}

robast_can_msgs::CanMessage create_drawer_feedback_can_msg()
{
  robast_can_msgs::CanMessage can_msg_drawer_feedback = can_db.can_messages.at(CAN_MSG_DRAWER_FEEDBACK);
  can_msg_drawer_feedback.can_signals.at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).data = DRAWER_ID;

  if (moving_average_drawer1_closed_pin > 0.9){
    can_msg_drawer_feedback.can_signals.at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_1_PUSHED).data = 1;
  } else {
    can_msg_drawer_feedback.can_signals.at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_1_PUSHED).data = 0;
  }

  if (moving_average_sensor_lock1_pin > 0.9){
    can_msg_drawer_feedback.can_signals.at(CAN_SIGNAL_IS_LOCK_SWITCH_1_PUSHED).data = 1;
  } else {
    can_msg_drawer_feedback.can_signals.at(CAN_SIGNAL_IS_LOCK_SWITCH_1_PUSHED).data = 0;
  }

  if (moving_average_drawer2_closed_pin > 0.9){
    can_msg_drawer_feedback.can_signals.at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_2_PUSHED).data = 1;
  } else {
    can_msg_drawer_feedback.can_signals.at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_2_PUSHED).data = 0;
  }

  if (moving_average_sensor_lock2_pin > 0.9){
    can_msg_drawer_feedback.can_signals.at(CAN_SIGNAL_IS_LOCK_SWITCH_2_PUSHED).data = 1;
  } else {
    can_msg_drawer_feedback.can_signals.at(CAN_SIGNAL_IS_LOCK_SWITCH_2_PUSHED).data = 0;
  }

  return can_msg_drawer_feedback;
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
  if(!digitalRead(MCP2515_INT))                         // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rx_msg_id, &rx_msg_dlc, rx_data_buf);

    std::optional<robast_can_msgs::CanMessage> can_message = robast_can_msgs::decode_can_message(rx_msg_id, rx_data_buf, rx_msg_dlc, can_db.can_messages); 

    if (can_message.has_value())
    {
      handle_CAN_msg(can_message.value());      
    }
    else
    {
      Serial.println("There is no CAN Message available in the CAN Database that corresponds to the msg id: ");
      Serial.print(rx_msg_id, HEX);
    }
  }

  
  handle_reading_sensors();

  unsigned long current_millis = millis();
  if (current_millis - previous_millis >= interval_drawer_feedback_in_ms)
  {
    previous_millis = current_millis;
    
    robast_can_msgs::CanMessage can_msg_drawer_feedback = create_drawer_feedback_can_msg();

    std::optional<robast_can_msgs::CanFrame> can_frame = robast_can_msgs::encode_can_message_into_can_frame(can_msg_drawer_feedback, can_db.can_messages);

    if (can_frame.has_value())
    {
      byte sndStat = CAN0.sendMsgBuf(can_frame.value().id, 0, can_frame.value().dlc, can_frame.value().data);
      if(sndStat == CAN_OK){
        Serial.println("Message Sent Successfully!");
      } else {
        Serial.print("Error Sending Message... CAN Status is: ");
        Serial.println(sndStat);
      }
    }
  }
}


