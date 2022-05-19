#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <FastLED.h>

#include "robast_can_msgs/can_db/can_db.h"

#define LOCK_POWER_PIN GPIO_NUM_21
#define LOCK_SENSOR_PIN GPIO_NUM_19
#define LED_POWER_PIN GPIO_NUM_18
#define LED_PIXEL_PIN GPIO_NUM_2
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

#define NUM_LEDS 19 // number of LEDs for LED strip

CRGBArray<NUM_LEDS> leds;

CAN_device_t CAN_cfg;               // CAN Config
unsigned long previousMillis = 0;   // will store last time a CAN Message was send
const int interval = 50;          // interval at which send CAN Messages (milliseconds)
const int rx_queue_size = 10;       // Receive Queue size

void can_setup() {
  CAN_cfg.speed = CAN_SPEED_125KBPS;
  CAN_cfg.tx_pin_id = CAN_TX_PIN;
  CAN_cfg.rx_pin_id = CAN_RX_PIN;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  // Init CAN Module
  ESP32Can.CANInit();
}


void can_loop() {
  CAN_frame_t rx_frame;

  unsigned long currentMillis = millis();

  // Receive next CAN frame from queue
  if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {

    if (rx_frame.FIR.B.FF == CAN_frame_std) {
      Serial.println("New standard frame");
    }
    else {
      Serial.println("New extended frame");
    }

    uint64_t can_data = 0;
    if (rx_frame.FIR.B.RTR == CAN_RTR) {
      printf(" RTR from 0x%08X, DLC %d\r\n", rx_frame.MsgID,  rx_frame.FIR.B.DLC);
    }
    else {
      printf(" from 0x%08X, DLC %d, Data ", rx_frame.MsgID,  rx_frame.FIR.B.DLC);
      printf("\n");
      printf("can_data: 0x%02X ",can_data);
      printf("\n");
    }
  }

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = 0x001;
    tx_frame.FIR.B.DLC = 8;
    tx_frame.data.u8[0] = 0x02;
    tx_frame.data.u8[1] = 0x02;
    tx_frame.data.u8[2] = 0x02;
    tx_frame.data.u8[3] = 0x02;
    tx_frame.data.u8[4] = 0x04;
    tx_frame.data.u8[5] = 0x04;
    tx_frame.data.u8[6] = 0x04;
    tx_frame.data.u8[7] = 0x04;
    // Serial.println("CAN Message to be sent!");
    ESP32Can.CANWriteFrame(&tx_frame);
    Serial.println("CAN Message sent!");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Basic Demo - ESP32 Drawer Controller");

  pinMode(LED_POWER_PIN, OUTPUT);
  pinMode(LOCK_POWER_PIN, OUTPUT);
  pinMode(LOCK_SENSOR_PIN, INPUT);

  FastLED.addLeds<NEOPIXEL,2>(leds, NUM_LEDS);
  FastLED.setBrightness(100);

  digitalWrite(LED_POWER_PIN, HIGH);

  can_setup();
  Serial.println("Setup finished!");
}

void loop() {
  can_loop();

  // bool lock_is_closed = !digitalRead(LOCK_SENSOR_PIN);

  // if (lock_is_closed) {
  //   // Serial.println("Lock is closed!");
  //   for(int i = 0; i < NUM_LEDS; i++) {   
  //     leds[i] = CRGB::Green;
  //   }
  //   FastLED.show();
  // }
  // else {
  //   // Serial.println("Lock is open!");
  //   for(int i = 0; i < NUM_LEDS; i++) {   
  //     leds[i] = CRGB::Red;
  //   }
  //   FastLED.show();
  // }
}



