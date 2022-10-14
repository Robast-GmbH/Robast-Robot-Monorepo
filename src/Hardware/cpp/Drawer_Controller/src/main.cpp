#include <Arduino.h>
#include <mcp_can.h>
#include <FastLED.h>

#include "pinout_defines.h"

#include "can_db.hpp"
#include "can_helper.h"

/*********************************************************************************************************
  GLOBAL VARIABLES AND CONSTANTS
*********************************************************************************************************/

#define DRAWER_CONTROLLER_ID 1 //TODO: Every DRAWER_CONTROLLER needs to have his own id

#define NUM_LEDS 25 // number of LEDs for LED strip
#define MIDDLE_LED 13 // address of the middle LED, which is important for running LED mode

CRGBArray<NUM_LEDS> leds;
uint8_t led_target_red;
uint8_t led_target_green;
uint8_t led_target_blue;
uint8_t led_target_brightness;
uint8_t led_target_brightness_fade_on_fade_off;
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

unsigned long previous_millis_drawer_status_fb = 0;
#define DEFAULT_INTERVAL_DRAWER_FEEDBACK_IN_MS 1000
unsigned long interval_drawer_feedback_in_ms = DEFAULT_INTERVAL_DRAWER_FEEDBACK_IN_MS;
bool broadcast_feedback = false;

float moving_average_sensor_lock1_pin = 0;
float moving_average_drawer1_closed_pin = 0;
float moving_average_sensor_lock2_pin = 0;
float moving_average_drawer2_closed_pin = 0;

// flags to store which state the locks should have
bool open_lock_1 = false;
bool open_lock_2 = false;
// flagt to store state of the lock of the previous step
bool open_lock_1_previous_step = false;
bool open_lock_2_previous_step = false;
// flag to indicate that lock state needs to change
bool change_lock_1_state = false;
bool change_lock_2_state = false;

// the time in ms the lock mechanism needs to open resp. close the lock
#define LOCK_MECHANISM_TIME 700 // according to the datasheet a minimum of 600ms is required
unsigned long previous_millis_open_lock_1 = 0;
unsigned long previous_millis_open_lock_2 = 0;

hw_timer_t * fading_up_timer = NULL;
portMUX_TYPE fading_up_timer_mux = portMUX_INITIALIZER_UNLOCKED;

hw_timer_t * running_led_timer = NULL;
portMUX_TYPE running_led_timer_mux = portMUX_INITIALIZER_UNLOCKED;

volatile uint8_t running_led_offset_from_middle = 0; // this variable controls which LED is currently shining for the running LED mode
#define NUM_OF_LED_SHADOWS 3 // Number of "shadow" LEDs for running LED. At the moment you need to do a few more changes to increase the number of shadow LEDs, in the future it should only be this define

/*********************************************************************************************************
  FUNCTIONS
*********************************************************************************************************/

void IRAM_ATTR on_timer_for_fading()
{
  if (led_target_brightness > led_current_brightness)
  {
    portENTER_CRITICAL_ISR(&fading_up_timer_mux);
    led_current_brightness++;
    portEXIT_CRITICAL_ISR(&fading_up_timer_mux);
  }

  if (led_target_brightness < led_current_brightness)
  {
    portENTER_CRITICAL_ISR(&fading_up_timer_mux);
    led_current_brightness--;
    portEXIT_CRITICAL_ISR(&fading_up_timer_mux);
  }
}

void IRAM_ATTR on_timer_for_running_led()
{
  if ((MIDDLE_LED - running_led_offset_from_middle) >= 0 - NUM_OF_LED_SHADOWS)
  {
    portENTER_CRITICAL_ISR(&running_led_timer_mux);
    running_led_offset_from_middle++;
    portEXIT_CRITICAL_ISR(&running_led_timer_mux);
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

  open_lock_1 = false;
  open_lock_2 = false;
}

void led_init_mode()
{
  led_target_red = 0;
  led_target_green = 155;
  led_target_blue = 155;
  led_target_brightness = 25;
}

void initialize_led_strip(void)
{
  FastLED.addLeds<NEOPIXEL,LED_PIXEL_PIN>(leds, NUM_LEDS);
  led_current_brightness = 0;
  led_target_brightness = 0;
  led_init_mode();
}

void initialize_timer(void)
{
  fading_up_timer = timerBegin(0, 80, true); // The base signal of the ESP32 has a frequency of 80Mhz -> prescaler 80 makes it 1Mhz
  timerAttachInterrupt(fading_up_timer, &on_timer_for_fading, true);
  timerAlarmWrite(fading_up_timer, 3000, true); // With the alarm_value of 3000 the interrupt will be triggert 333/s
  timerAlarmEnable(fading_up_timer);

  running_led_timer = timerBegin(1, 80, true); // The base signal of the ESP32 has a frequency of 80Mhz -> prescaler 80 makes it 1Mhz
  timerAttachInterrupt(running_led_timer, &on_timer_for_running_led, true);
  timerAlarmWrite(running_led_timer, 50000, true); // 50000 is a good value. This defines how fast the LED will "run". Higher values will decrease the running speed.
  timerAlarmEnable(running_led_timer);
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

void open_lock(uint8_t lock_id)
{
  if (lock_id == 1)
  {
    digitalWrite(PWR_CLOSE_LOCK1_PIN, LOW);
    digitalWrite(PWR_OPEN_LOCK1_PIN, HIGH);
  }
  if (lock_id == 2)
  {
    digitalWrite(PWR_CLOSE_LOCK2_PIN, LOW);
    digitalWrite(PWR_OPEN_LOCK2_PIN, HIGH);
  }
}

void close_lock(uint8_t lock_id)
{
  if (lock_id == 1)
  {
    digitalWrite(PWR_OPEN_LOCK1_PIN, LOW);
    digitalWrite(PWR_CLOSE_LOCK1_PIN, HIGH);
  }
  if (lock_id == 2)
  {
    digitalWrite(PWR_OPEN_LOCK2_PIN, LOW);
    digitalWrite(PWR_CLOSE_LOCK2_PIN, HIGH);
  }
}

void set_lock_output_low(uint8_t lock_id)
{
  if (lock_id == 1)
  {
    digitalWrite(PWR_OPEN_LOCK1_PIN, LOW);
    digitalWrite(PWR_CLOSE_LOCK1_PIN, LOW);
  }
  if (lock_id == 2)
  {
    digitalWrite(PWR_OPEN_LOCK2_PIN, LOW);
    digitalWrite(PWR_CLOSE_LOCK2_PIN, LOW);
  }
}

void handle_lock_status(robast_can_msgs::CanMessage can_message)
{
  if (can_message.get_can_signals().at(CAN_SIGNAL_OPEN_LOCK_1).get_data() == CAN_DATA_OPEN_LOCK)
  {
    open_lock_1 = true;
    activate_drawer_feedback_broadcast();
  }
  if (can_message.get_can_signals().at(CAN_SIGNAL_OPEN_LOCK_1).get_data() == CAN_DATA_CLOSE_LOCK)
  {
    open_lock_1 = false;
  }

  if (can_message.get_can_signals().at(CAN_SIGNAL_OPEN_LOCK_2).get_data() == CAN_DATA_OPEN_LOCK)
  {
    open_lock_2 = true;
    activate_drawer_feedback_broadcast();
  }
  if (can_message.get_can_signals().at(CAN_SIGNAL_OPEN_LOCK_2).get_data() == CAN_DATA_CLOSE_LOCK)
  {
    open_lock_2 = false;
  }
}

void led_standard_mode()
{
  for(int i = 0; i < NUM_LEDS; i++)
  {   
    leds[i] = CRGB(led_target_red, led_target_green, led_target_blue);
  }
  led_current_red = led_target_red;
  led_current_green = led_target_green;
  led_current_blue = led_target_blue;
  led_current_brightness = led_target_brightness;
  FastLED.setBrightness(led_target_brightness);
  FastLED.show();
}

void led_fade_on_mode()
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
    led_current_blue = led_target_blue;
    FastLED.setBrightness(led_current_brightness);
    FastLED.show();
  }  
}

void led_closing_drawer_mode()
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
    led_current_blue = led_target_blue;
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
  deactivate_drawer_feedback_broadcast();
}

void led_fade_on_fade_off_mode()
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
    led_current_blue = led_target_blue;
    FastLED.setBrightness(led_current_brightness);
    FastLED.show();
  }

  // Mind that the variable led_current_brightness is increased/decreased in a seperate interrupt
  if (led_current_brightness == 0)
  {
    led_target_brightness = led_target_brightness_fade_on_fade_off;
  }
  else if (led_current_brightness == led_target_brightness_fade_on_fade_off)
  {
    led_target_brightness = 0;
  }
}

void select_led_strip_mode(robast_can_msgs::CanMessage can_message)
{
  led_target_red = can_message.get_can_signals().at(CAN_SIGNAL_LED_RED).get_data();
  led_target_green = can_message.get_can_signals().at(CAN_SIGNAL_LED_GREEN).get_data();
  led_target_blue = can_message.get_can_signals().at(CAN_SIGNAL_LED_BLUE).get_data();
  led_target_brightness = can_message.get_can_signals().at(CAN_SIGNAL_LED_BRIGHTNESS).get_data();
  led_mode = can_message.get_can_signals().at(CAN_SIGNAL_LED_MODE).get_data();

  switch (led_mode)
    {
      case 0:
        // standard mode
        break;

      case 1:
        // fade on mode
        timerAlarmWrite(fading_up_timer, 3000, true); // With the alarm_value of 3000 the interrupt will be triggert 333/s    
        portENTER_CRITICAL(&fading_up_timer_mux);
        led_current_brightness = 0;
        portEXIT_CRITICAL(&fading_up_timer_mux);  
        break;

      case 2:
        // led closing drawer mode
        portENTER_CRITICAL(&running_led_timer_mux);
        running_led_offset_from_middle = 0;
        portEXIT_CRITICAL(&running_led_timer_mux);
        break;

      case 3:
        // led fade on + fade off mode
        timerAlarmWrite(fading_up_timer, 10000, true); // fade on + fade off should be more slowly than only fading on, so choose a bigger value for the alarm_value
        portENTER_CRITICAL(&fading_up_timer_mux);
        led_current_brightness = 0;
        portEXIT_CRITICAL(&fading_up_timer_mux);
        led_target_brightness_fade_on_fade_off = led_target_brightness;
      
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

      select_led_strip_mode(can_message);
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

void handle_lock_control(void)
{
  // Mind that the state for open_lock_1 and open_lock_2 is changed in the handle_lock_status function when a CAN msg is received
  change_lock_1_state = open_lock_1 == open_lock_1_previous_step ? false : true;
  change_lock_2_state = open_lock_2 == open_lock_2_previous_step ? false : true;

  unsigned long current_millis_open_lock_1 = millis();
  unsigned long current_millis_open_lock_2 = millis();

  if (change_lock_1_state && (current_millis_open_lock_1 - previous_millis_open_lock_1 >= LOCK_MECHANISM_TIME))
  {
    open_lock_1_previous_step = open_lock_1;
    previous_millis_open_lock_1 = current_millis_open_lock_1;
    open_lock_1 ? open_lock(1) : close_lock(1);
  }
  else if (!change_lock_1_state && (current_millis_open_lock_1 - previous_millis_open_lock_1 >= LOCK_MECHANISM_TIME))
  {
    // this makes sure, there is only a 5V pulse with the duration of LOCK_MECHANISM_TIME on the respective input of the lock
    set_lock_output_low(1);
  }

  if (change_lock_2_state && (current_millis_open_lock_2 - previous_millis_open_lock_2 >= LOCK_MECHANISM_TIME))
  {
    open_lock_2_previous_step = open_lock_2;
    previous_millis_open_lock_2 = current_millis_open_lock_2;
    open_lock_2 ? open_lock(2) : close_lock(2);
  }
  else if (!change_lock_2_state && (current_millis_open_lock_2 - previous_millis_open_lock_2 >= LOCK_MECHANISM_TIME))
  {
    // this makes sure, there is only a 5V pulse with the duration of LOCK_MECHANISM_TIME on the respective input of the lock
    set_lock_output_low(1);
  }
}

void handle_led_control(void)
{
  switch (led_mode)
    {
      case 0:
        led_standard_mode();
        break;

      case 1:
        led_fade_on_mode();
        break;

      case 2:
        led_closing_drawer_mode();
        break;

      case 3:
        led_fade_on_fade_off_mode();
        break;
      
      default:
        led_standard_mode();
        break;
    }
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

  initialize_locks();

  initialize_led_strip();

  initialize_timer();
}


/*********************************************************************************************************
  LOOP
*********************************************************************************************************/

void loop()
{
  handle_receiving_can_msg();

  handle_lock_control();

  handle_led_control();
  
  handle_reading_sensors();

  handle_sending_drawer_status_feedback();
}


