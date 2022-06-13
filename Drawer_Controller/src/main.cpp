#include <Arduino.h>
#include <CAN_config.h>
#include <FastLED.h>

#define OPEN_LOCK1_PIN GPIO_NUM_22
#define CLOSE_LOCK1_PIN GPIO_NUM_21
#define LOCK_SENSOR_PIN GPIO_NUM_39
#define LED_PIXEL_PIN GPIO_NUM_13
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

#define NUM_LEDS 19 // number of LEDs for LED strip

CRGBArray<NUM_LEDS> leds;

bool lock_is_closed_last_loop;
bool fade_up;
uint8_t brightness;
uint8_t brightness_maximum = 200;


void setup() {
  Serial.begin(115200);

  pinMode(OPEN_LOCK1_PIN, OUTPUT);
  pinMode(CLOSE_LOCK1_PIN, OUTPUT);
  pinMode(LOCK_SENSOR_PIN, INPUT);

  FastLED.addLeds<NEOPIXEL,LED_PIXEL_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(100);

  lock_is_closed_last_loop = false;
  fade_up = true;
  brightness = 0;
  digitalWrite(OPEN_LOCK1_PIN, LOW);
  digitalWrite(CLOSE_LOCK1_PIN, LOW);
}

void loop() {

  bool lock_is_closed = !digitalRead(LOCK_SENSOR_PIN);

  if (lock_is_closed) {
    // Serial.println("Lock is closed!");
    for(int i = 0; i < NUM_LEDS; i++) {   
      leds[i] = CRGB::Green;
    }
    FastLED.show();

    digitalWrite(OPEN_LOCK1_PIN, LOW);
    digitalWrite(CLOSE_LOCK1_PIN, HIGH);
  }
  else {
    // Serial.println("Lock is open!");
    for(int i = 0; i < NUM_LEDS; i++) {   
      leds[i] = CRGB::Red;
    }
    FastLED.show();

    digitalWrite(CLOSE_LOCK1_PIN, LOW);
    digitalWrite(OPEN_LOCK1_PIN, HIGH);
  }
}