#include <Arduino.h>
#include <mcp_can.h>

#include "pinout_defines.h"
#include "lock.hpp"
#include "led_strip.hpp"
#include "can/can_db.hpp"
#include "can/can_helper.h"


/*********************************************************************************************************
  GLOBAL VARIABLES AND CONSTANTS
*********************************************************************************************************/

#define DRAWER_CONTROLLER_ID 1 //TODO: Every DRAWER_CONTROLLER needs to have his own id

MCP_CAN CAN0(SPI_CS);

lock::Lock LOCK_1 = lock::Lock();
lock::Lock LOCK_2 = lock::Lock();

robast_can_msgs::CanDb can_db = robast_can_msgs::CanDb();

long unsigned int rx_msg_id;
uint8_t rx_msg_dlc = 0;
uint8_t rx_data_buf[8];

unsigned long previous_millis_drawer_status_fb = 0;
#define DEFAULT_INTERVAL_DRAWER_FEEDBACK_IN_MS 1000
unsigned long interval_drawer_feedback_in_ms = DEFAULT_INTERVAL_DRAWER_FEEDBACK_IN_MS;
bool broadcast_feedback = false;

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
  if(CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK)
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

void activate_drawer_feedback_broadcast(void)
{
  broadcast_feedback = true;
  interval_drawer_feedback_in_ms = 50;
}

void deactivate_drawer_feedback_broadcast(void)
{
  broadcast_feedback = false;
  interval_drawer_feedback_in_ms = DEFAULT_INTERVAL_DRAWER_FEEDBACK_IN_MS;
}

void handle_lock_status(robast_can_msgs::CanMessage can_message)
{
  if (can_message.get_can_signals().at(CAN_SIGNAL_OPEN_LOCK_1).get_data() == CAN_DATA_OPEN_LOCK)
  {
    LOCK_1.set_open_lock_current_step(true);
    activate_drawer_feedback_broadcast();
  }
  if (can_message.get_can_signals().at(CAN_SIGNAL_OPEN_LOCK_1).get_data() == CAN_DATA_CLOSE_LOCK)
  {
    LOCK_1.set_open_lock_current_step(false);
  }

  if (can_message.get_can_signals().at(CAN_SIGNAL_OPEN_LOCK_2).get_data() == CAN_DATA_OPEN_LOCK)
  {
    LOCK_2.set_open_lock_current_step(true);
    activate_drawer_feedback_broadcast();
  }
  if (can_message.get_can_signals().at(CAN_SIGNAL_OPEN_LOCK_2).get_data() == CAN_DATA_CLOSE_LOCK)
  {
    LOCK_2.set_open_lock_current_step(false);
  }
}

void debug_prints(robast_can_msgs::CanMessage can_message)
{
  Serial.print("Standard ID: ");
  Serial.print(rx_msg_id, HEX);
  Serial.print(" rx_dlc: ");
  Serial.print(uint8_t(rx_msg_dlc), DEC);
  Serial.print(" DRAWER ID: ");
  Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).get_data(), HEX);
  Serial.print(" CAN_SIGNAL_OPEN_LOCK_1: ");
  Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_OPEN_LOCK_1).get_data(), BIN);
  Serial.print(" CAN_SIGNAL_OPEN_LOCK_2: ");
  Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_OPEN_LOCK_2).get_data(), BIN);
  Serial.print(" LED RED: ");
  Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_LED_RED).get_data(), DEC);
  Serial.print(" LED GREEN: ");
  Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_LED_GREEN).get_data(), DEC);
  Serial.print(" LED BLUE: ");
  Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_LED_BLUE).get_data(), DEC);
  Serial.print(" LED BRIGHTNESS: ");
  Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_LED_BRIGHTNESS).get_data(), DEC);
  Serial.print(" LED MODE: ");
  Serial.println(can_message.get_can_signals().at(CAN_SIGNAL_LED_MODE).get_data(), DEC);
}

void handle_can_msg(robast_can_msgs::CanMessage can_message)
{
  if (can_message.get_id() == CAN_ID_DRAWER_USER_ACCESS)
  {
    if (can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).get_data() == DRAWER_CONTROLLER_ID)
    {
      handle_lock_status(can_message);

      led_strip::select_led_strip_mode(can_message);
    }

    debug_prints(can_message);
  }
}

void handle_receiving_can_msg()
{
  if(!digitalRead(MCP2515_INT)) // If CAN0_INT pin is low, read receive buffer
  {  
    Serial.println("Received CAN message!");

    CAN0.readMsgBuf(&rx_msg_id, &rx_msg_dlc, rx_data_buf);

    std::optional<robast_can_msgs::CanMessage> can_message = robast_can_msgs::decode_can_message(rx_msg_id, rx_data_buf, rx_msg_dlc, can_db.can_messages); 

    if (can_message.has_value())
    {
      handle_can_msg(can_message.value());      
    }
    else
    {
      Serial.println("There is no CAN Message available in the CAN Database that corresponds to the msg id: ");
      Serial.print(rx_msg_id, HEX);
    }
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
  std::vector can_signals_drawer_feedback = can_msg_drawer_feedback.get_can_signals();

  can_signals_drawer_feedback.at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).set_data(DRAWER_CONTROLLER_ID);

  if (moving_average_drawer1_closed_pin > 0.9){
    can_signals_drawer_feedback.at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_1_PUSHED).set_data(1);
  } else {
    can_signals_drawer_feedback.at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_1_PUSHED).set_data(0);
  }

  if (moving_average_sensor_lock1_pin > 0.9){
    can_signals_drawer_feedback.at(CAN_SIGNAL_IS_LOCK_SWITCH_1_PUSHED).set_data(1);
  } else {
    can_signals_drawer_feedback.at(CAN_SIGNAL_IS_LOCK_SWITCH_1_PUSHED).set_data(0);
  }

  if (moving_average_drawer2_closed_pin > 0.9){
    can_signals_drawer_feedback.at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_2_PUSHED).set_data(1);
  } else {
    can_signals_drawer_feedback.at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_2_PUSHED).set_data(0);
  }

  if (moving_average_sensor_lock2_pin > 0.9){
    can_signals_drawer_feedback.at(CAN_SIGNAL_IS_LOCK_SWITCH_2_PUSHED).set_data(1);
  } else {
    can_signals_drawer_feedback.at(CAN_SIGNAL_IS_LOCK_SWITCH_2_PUSHED).set_data(0);
  }

  can_msg_drawer_feedback.set_can_signals(can_signals_drawer_feedback);

  return can_msg_drawer_feedback;
}

void sending_drawer_status_feedback(void)
{
  robast_can_msgs::CanMessage can_msg_drawer_feedback = create_drawer_feedback_can_msg();

  try
  {
    robast_can_msgs::CanFrame can_frame = robast_can_msgs::encode_can_message_into_can_frame(can_msg_drawer_feedback, can_db.can_messages);
    byte sndStat = CAN0.sendMsgBuf(can_frame.get_id(), 0, can_frame.get_dlc(), can_frame.get_data());
    if(sndStat == CAN_OK)
    {
      Serial.println("Message Sent Successfully!");
    }
    else
    {
      Serial.print("Error Sending Message... CAN Status is: ");
      Serial.println(sndStat);
    }  
  }
  catch (const std::invalid_argument& exception)
  {
    Serial.print("Exception accurred while encoding CAN message into can frame. Exception message: ");
    Serial.println(exception.what());
  }
}

void handle_sending_drawer_status_feedback(void)
{
  unsigned long current_millis_drawer_status_fb = millis();
  if (broadcast_feedback && (current_millis_drawer_status_fb - previous_millis_drawer_status_fb >= interval_drawer_feedback_in_ms))
  {
    previous_millis_drawer_status_fb = current_millis_drawer_status_fb;
    
    sending_drawer_status_feedback();
  }
}


/*********************************************************************************************************
  SETUP
*********************************************************************************************************/

void setup()
{
  Serial.begin(115200);

  initialize_voltage_translator();  

  initialize_can_controller();

  LOCK_1.initialize_locks(PWR_OPEN_LOCK1_PIN, PWR_CLOSE_LOCK1_PIN, SENSOR_LOCK1_PIN, SENSOR_DRAWER1_CLOSED_PIN);
  LOCK_2.initialize_locks(PWR_OPEN_LOCK2_PIN, PWR_CLOSE_LOCK2_PIN, SENSOR_LOCK2_PIN, SENSOR_DRAWER2_CLOSED_PIN);

  led_strip::initialize_led_strip();
}


/*********************************************************************************************************
  LOOP
*********************************************************************************************************/

void loop()
{
  handle_receiving_can_msg();

  LOCK_1.handle_lock_control();
  LOCK_2.handle_lock_control();

  led_strip::handle_led_control();
  
  handle_reading_sensors();

  handle_sending_drawer_status_feedback();
}


