#include <Arduino.h>
#include <CAN_config.h>
#include <FastLED.h>

#define LOCK_POWER_PIN GPIO_NUM_21
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

  pinMode(LOCK_POWER_PIN, OUTPUT);
  pinMode(LOCK_SENSOR_PIN, INPUT);

  FastLED.addLeds<NEOPIXEL,LED_PIXEL_PIN>(leds, NUM_LEDS);
  // FastLED.setBrightness(191);

  lock_is_closed_last_loop = false;
  fade_up = true;
  brightness = 0;
}

void loop() {

  bool lock_is_closed = !digitalRead(LOCK_SENSOR_PIN);

  if (lock_is_closed != lock_is_closed_last_loop) {
    fade_up = true;
    brightness = 0;
  }

  lock_is_closed_last_loop = lock_is_closed;

  if (fade_up) {
    if (brightness == brightness_maximum)
		{
			fade_up = false;
		}
    else {
      brightness = brightness + 1;
    }
    FastLED.setBrightness(brightness);

    delay(20);
  }
  
  if (lock_is_closed) {

    Serial.println("Lock is closed!");
    for(int i = 0; i < NUM_LEDS; i++) {   
      leds[i] = CRGB::SeaGreen;
    }
    FastLED.show();
  }
  else {
    Serial.println("Lock is open!");
    for(int i = 0; i < NUM_LEDS; i++) {   
      leds[i] = CRGB::White;
    }
    FastLED.show();
  }
}