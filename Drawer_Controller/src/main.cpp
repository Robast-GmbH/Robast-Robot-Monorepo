#include <Arduino.h>
#include <CAN_config.h>
#include <FastLED.h>

#define LOCK_POWER_PIN GPIO_NUM_21
#define LOCK_SENSOR_PIN GPIO_NUM_19
#define LED_POWER_PIN GPIO_NUM_18
#define LED_PIXEL_PIN GPIO_NUM_2
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

#define NUM_LEDS 19 // number of LEDs for LED strip

CRGBArray<NUM_LEDS> leds;

void setup() {
  Serial.begin(115200);

  pinMode(LED_POWER_PIN, OUTPUT);
  pinMode(LOCK_POWER_PIN, OUTPUT);
  pinMode(LOCK_SENSOR_PIN, INPUT);

  FastLED.addLeds<NEOPIXEL,2>(leds, NUM_LEDS);
  FastLED.setBrightness(100);

  digitalWrite(LED_POWER_PIN, HIGH);
}

void loop() {

  bool lock_is_closed = !digitalRead(LOCK_SENSOR_PIN);

  if (lock_is_closed) {
    // Serial.println("Lock is closed!");
    for(int i = 0; i < NUM_LEDS; i++) {   
      leds[i] = CRGB::Green;
    }
    FastLED.show();
  }
  else {
    // Serial.println("Lock is open!");
    for(int i = 0; i < NUM_LEDS; i++) {   
      leds[i] = CRGB::Red;
    }
    FastLED.show();
  }
}