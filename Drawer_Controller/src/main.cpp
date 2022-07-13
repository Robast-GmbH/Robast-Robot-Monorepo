#include <Arduino.h>
#include <mcp_can.h>
#include <FastLED.h>

#include "robast_can_msgs/can_db.h"
#include "robast_can_msgs/can_helper.h"

/*********************************************************************************************************
  GLOBAL VARIABLES AND CONSTANTS
*********************************************************************************************************/

#define DRAWER_CONTROLLER_ID 1 //TODO: Every DRAWER_CONTROLLER needs to have his own id

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
#define MIDDLE_LED 9 // adress of the middle LED

CRGBArray<NUM_LEDS> leds;
uint8_t led_target_red;
uint8_t led_target_green;
uint8_t led_target_blue;
uint8_t led_target_brightness;
uint8_t led_current_red;
uint8_t led_current_green;
uint8_t led_current_blue;
volatile uint8_t led_current_brightness;
uint8_t led_mode;

MCP_CAN CAN0(SPI_CS);

robast_can_msgs::CanDb can_db = robast_can_msgs::CanDb();

long unsigned int rx_msg_id;
uint8_t rx_msg_dlc = 0;
uint8_t rx_data_buf[8];

unsigned long previous_millis = 0;
#define DEFAULT_INTERVAL_DRAWER_FEEDBACK_IN_MS 1000
unsigned long interval_drawer_feedback_in_ms = DEFAULT_INTERVAL_DRAWER_FEEDBACK_IN_MS;
bool broadcast_feedback = false;

float moving_average_sensor_lock1_pin = 0;
float moving_average_drawer1_closed_pin = 0;
float moving_average_sensor_lock2_pin = 0;
float moving_average_drawer2_closed_pin = 0;

hw_timer_t * fading_up_timer = NULL;
portMUX_TYPE fading_up_timer_mux = portMUX_INITIALIZER_UNLOCKED;

hw_timer_t * running_led_timer = NULL;
portMUX_TYPE running_led_timer_mux = portMUX_INITIALIZER_UNLOCKED;

volatile uint8_t running_led_offset_from_middle = 0;
#define NUM_OF_LED_SHADOWS 3

/*********************************************************************************************************
  FUNCTIONS
*********************************************************************************************************/

void IRAM_ATTR on_timer_for_fading_up()
{
  if (led_target_brightness > led_current_brightness)
  {
    portENTER_CRITICAL_ISR(&running_led_timer_mux);
    led_current_brightness++;
    portEXIT_CRITICAL_ISR(&running_led_timer_mux);
  }

  if (led_target_brightness < led_current_brightness)
  {
    portENTER_CRITICAL_ISR(&running_led_timer_mux);
    led_current_brightness--;
    portEXIT_CRITICAL_ISR(&running_led_timer_mux);
  }
}

void IRAM_ATTR on_timer_for_running_led()
{
  if ((MIDDLE_LED - running_led_offset_from_middle) >= 0 - NUM_OF_LED_SHADOWS)
  {
    portENTER_CRITICAL_ISR(&fading_up_timer_mux);
    running_led_offset_from_middle++;
    portEXIT_CRITICAL_ISR(&fading_up_timer_mux);
  }
}

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
  led_current_brightness = 0;
  led_target_brightness = 0;
}

void initialize_timer(void)
{
  fading_up_timer = timerBegin(0, 80, true); // The base signal of the ESP32 has a frequency of 80Mhz -> prescaler 80 makes it 1Mhz
  timerAttachInterrupt(fading_up_timer, &on_timer_for_fading_up, true);
  timerAlarmWrite(fading_up_timer, 3000, true); // With the alarm_value of 3000 the interrupt will be triggert 333/s
  timerAlarmEnable(fading_up_timer);

  running_led_timer = timerBegin(1, 80, true); // The base signal of the ESP32 has a frequency of 80Mhz -> prescaler 80 makes it 1Mhz
  timerAttachInterrupt(running_led_timer, &on_timer_for_running_led, true);
  timerAlarmWrite(running_led_timer, 50000, true); // 50000 is a good value
  timerAlarmEnable(running_led_timer);
}

void handle_locks(robast_can_msgs::CanMessage can_message)
{
  if (can_message.can_signals.at(CAN_SIGNAL_OPEN_LOCK_1).data == CAN_DATA_OPEN_LOCK)
  {
    // Once the opening of a lock is triggert, we want to activate the drawer feedback broadcast
    broadcast_feedback = true;
    interval_drawer_feedback_in_ms = 50;
    digitalWrite(PWR_CLOSE_LOCK1_PIN, LOW);
    digitalWrite(PWR_OPEN_LOCK1_PIN, HIGH);
  }
  if (can_message.can_signals.at(CAN_SIGNAL_OPEN_LOCK_1).data == CAN_DATA_CLOSE_LOCK)
  {
    digitalWrite(PWR_OPEN_LOCK1_PIN, LOW);
    digitalWrite(PWR_CLOSE_LOCK1_PIN, HIGH);
  }

  if (can_message.can_signals.at(CAN_SIGNAL_OPEN_LOCK_2).data == CAN_DATA_OPEN_LOCK)
  {
    // Once the opening of a lock is triggert, we want to activate the drawer feedback broadcast
    broadcast_feedback = true;
    interval_drawer_feedback_in_ms = 50;
    digitalWrite(PWR_CLOSE_LOCK2_PIN, LOW);
    digitalWrite(PWR_OPEN_LOCK2_PIN, HIGH);
  }
  if (can_message.can_signals.at(CAN_SIGNAL_OPEN_LOCK_2).data == CAN_DATA_CLOSE_LOCK)
  {
    digitalWrite(PWR_OPEN_LOCK2_PIN, LOW);
    digitalWrite(PWR_CLOSE_LOCK2_PIN, HIGH);
  }
}

void LED_standard_mode()
{
  if (led_target_brightness != led_current_brightness ||
      led_target_red != led_current_red ||
      led_target_green != led_current_green ||
      led_target_blue != led_current_blue)
  {
    for(int i = 0; i < NUM_LEDS; i++)
    {   
      leds[i] = CRGB(led_target_red, led_target_green, led_target_blue);
    }
    led_current_red = led_target_red;
    led_current_green = led_target_green;
    led_current_blue = led_target_green;
    led_current_brightness = led_target_brightness;
    FastLED.setBrightness(led_target_brightness);
    FastLED.show();
  }
}

void LED_fade_on_mode()
{
  // Mind that the variable led_current_brightness is increased/decreased in a seperate interrupt
  if (led_target_brightness != led_current_brightness ||
      led_target_red != led_current_red ||
      led_target_green != led_current_green ||
      led_target_blue != led_current_blue)
  {
    for(int i = 0; i < NUM_LEDS; i++)
    {   
      leds[i] = CRGB(led_target_red, led_target_green, led_target_blue);
    }
    led_current_red = led_target_red;
    led_current_green = led_target_green;
    led_current_blue = led_target_green;
    FastLED.setBrightness(led_current_brightness);
    FastLED.show();
  }  
}

void LED_closing_drawer_mode()
{
  if ((MIDDLE_LED - running_led_offset_from_middle) >= 0 - NUM_OF_LED_SHADOWS)
  {
    for(int i = 0; i < NUM_LEDS; i++)
    {
      if ((i == (MIDDLE_LED - running_led_offset_from_middle)) || (i == (MIDDLE_LED + running_led_offset_from_middle)))
      {
        leds[i] = CRGB(led_target_red, led_target_green, led_target_blue);
      }
      // Create a shadow of running LED with less brightness
      else if ((running_led_offset_from_middle >= 1) && 
               ((i == (MIDDLE_LED - running_led_offset_from_middle + 1)) || (i == (MIDDLE_LED + running_led_offset_from_middle - 1))))
      {
        leds[i] = CRGB(led_target_red/2, led_target_green/2, led_target_blue/2);
      }
      // Create a shadow of running LED with less brightness
      else if ((running_led_offset_from_middle >= 2) && 
               ((i == (MIDDLE_LED - running_led_offset_from_middle + 2)) || (i == (MIDDLE_LED + running_led_offset_from_middle - 2))))
      {
        leds[i] = CRGB(led_target_red/3, led_target_green/3, led_target_blue/3);
      }
      // Create a shadow of running LED with less brightness
      else if ((running_led_offset_from_middle >= 3) && 
               ((i == (MIDDLE_LED - running_led_offset_from_middle + 3)) || (i == (MIDDLE_LED + running_led_offset_from_middle - 3))))
      {
        leds[i] = CRGB(led_target_red/4, led_target_green/4, led_target_blue/4);
      }
      else
      {
        leds[i] = CRGB(0, 0, 0);
      }
    }
    led_current_red = led_target_red;
    led_current_green = led_target_green;
    led_current_blue = led_target_green;
    led_current_brightness = led_target_brightness;
    FastLED.setBrightness(led_target_brightness);
    FastLED.show();
  }

  if ((MIDDLE_LED - running_led_offset_from_middle) < 0 - NUM_OF_LED_SHADOWS)
  {
    for(int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB(0, 0, 0);
    }
    FastLED.setBrightness(0);
    FastLED.show();
  }

  // Once closing the drawer is finished, set back the interval time to the default value and deactivate broadcast feedback
  interval_drawer_feedback_in_ms = DEFAULT_INTERVAL_DRAWER_FEEDBACK_IN_MS;
  broadcast_feedback = false;
}

void select_LED_strip_mode(robast_can_msgs::CanMessage can_message)
{
  led_target_red = can_message.can_signals.at(CAN_SIGNAL_LED_RED).data;
  led_target_green = can_message.can_signals.at(CAN_SIGNAL_LED_GREEN).data;
  led_target_blue = can_message.can_signals.at(CAN_SIGNAL_LED_BLUE).data;
  led_target_brightness = can_message.can_signals.at(CAN_SIGNAL_LED_BRIGHTNESS).data;
  led_mode = can_message.can_signals.at(CAN_SIGNAL_LED_MODE).data;

  switch (led_mode)
    {
      case 0:
        break;

      case 1:
        break;

      case 2:
        portENTER_CRITICAL(&running_led_timer_mux);
        running_led_offset_from_middle = 0;
        portEXIT_CRITICAL(&running_led_timer_mux);
      
      default:
        break;
    }
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
  Serial.print(can_message.can_signals.at(CAN_SIGNAL_LED_RED).data, DEC);
  Serial.print(" LED GREEN: ");
  Serial.print(can_message.can_signals.at(CAN_SIGNAL_LED_GREEN).data, DEC);
  Serial.print(" LED BLUE: ");
  Serial.print(can_message.can_signals.at(CAN_SIGNAL_LED_BLUE).data, DEC);
  Serial.print(" LED BRIGHTNESS: ");
  Serial.print(can_message.can_signals.at(CAN_SIGNAL_LED_BRIGHTNESS).data, DEC);
  Serial.print(" LED MODE: ");
  Serial.println(can_message.can_signals.at(CAN_SIGNAL_LED_MODE).data, DEC);
}

void handle_CAN_msg(robast_can_msgs::CanMessage can_message)
{
  if (can_message.id == CAN_ID_DRAWER_USER_ACCESS)
  {
    if (can_message.can_signals.at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).data == DRAWER_CONTROLLER_ID)
    {
      handle_locks(can_message);

      select_LED_strip_mode(can_message);

      debug_prints(can_message);
    }
  }
}

void handle_receiving_can_msg()
{
    Serial.println("Received CAN message!");

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
  can_msg_drawer_feedback.can_signals.at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).data = DRAWER_CONTROLLER_ID;

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

void handle_LED_control(void)
{
  switch (led_mode)
    {
      case 0:
        LED_standard_mode();
        break;

      case 1:
        LED_fade_on_mode();
        break;

      case 2:
        LED_closing_drawer_mode();
        break;
      
      default:
        LED_standard_mode();
        break;
    }
}

void sending_drawer_status_feedback(void)
{
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

  initialize_timer();
}


/*********************************************************************************************************
  LOOP
*********************************************************************************************************/

void loop()
{
  if(!digitalRead(MCP2515_INT)) // If CAN0_INT pin is low, read receive buffer
  {
    handle_receiving_can_msg();
  }

  handle_LED_control();
  
  handle_reading_sensors();

  unsigned long current_millis = millis();
  if (current_millis - previous_millis >= interval_drawer_feedback_in_ms && broadcast_feedback)
  {
    previous_millis = current_millis;
    
    sending_drawer_status_feedback();
  }
}


