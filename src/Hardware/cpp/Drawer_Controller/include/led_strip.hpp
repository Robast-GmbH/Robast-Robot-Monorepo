#if !defined(LED_STRIP_HPP)
#define LED_STRIP_HPP

#include <Arduino.h>
#include <FastLED.h>

#include "pinout_defines.h"
#include "can/can_db.hpp"
#include "can/can_helper.h"

namespace led_strip
{
    /*********************************************************************************************************
     GLOBAL VARIABLES AND CONSTANTS
    *********************************************************************************************************/
   #define NUM_OF_LEDS 25

    struct {
        uint8_t num_leds; // number of LEDs for LED strip
        uint8_t middle_led; // address of the middle LED, which is important for running LED mode
        uint8_t num_of_led_shadows; // Number of "shadow" LEDs for running LED. At the moment you need to do a few more changes to increase the number of shadow LEDs, in the future it should only be this define

        volatile uint8_t led_current_brightness;
        uint8_t led_current_red;
        uint8_t led_current_green;
        uint8_t led_current_blue;

        uint8_t led_target_brightness;
        uint8_t led_target_red;
        uint8_t led_target_green;
        uint8_t led_target_blue;
        uint8_t led_target_brightness_fade_on_fade_off;
        
        uint8_t led_mode;

        volatile uint8_t running_led_offset_from_middle; // this variable controls which LED is currently shining for the running LED mode
    } led_strip_1, led_strip_2; // currently we don't need led_strip_2, this is just here for example 

    CRGBArray<NUM_OF_LEDS> leds;

    hw_timer_t * fading_up_timer = NULL;
    portMUX_TYPE fading_up_timer_mux;

    hw_timer_t * running_led_timer = NULL;
    portMUX_TYPE running_led_timer_mux;

    /*********************************************************************************************************
     FUNCTIONS
    *********************************************************************************************************/

    void select_led_strip_mode(robast_can_msgs::CanMessage can_message)
    {
        led_strip_1.led_target_red = can_message.get_can_signals().at(CAN_SIGNAL_LED_RED).get_data();
        led_strip_1.led_target_green = can_message.get_can_signals().at(CAN_SIGNAL_LED_GREEN).get_data();
        led_strip_1.led_target_blue = can_message.get_can_signals().at(CAN_SIGNAL_LED_BLUE).get_data();
        led_strip_1.led_target_brightness = can_message.get_can_signals().at(CAN_SIGNAL_LED_BRIGHTNESS).get_data();
        led_strip_1.led_mode = can_message.get_can_signals().at(CAN_SIGNAL_LED_MODE).get_data();

        switch (led_strip_1.led_mode)
            {
            case 0:
                // standard mode
                break;

            case 1:
                // fade on mode
                timerAlarmWrite(fading_up_timer, 3000, true); // With the alarm_value of 3000 the interrupt will be triggert 333/s    
                portENTER_CRITICAL(&fading_up_timer_mux);
                led_strip_1.led_current_brightness = 0;
                portEXIT_CRITICAL(&fading_up_timer_mux);  
                break;

            case 2:
                // led closing drawer mode
                portENTER_CRITICAL(&running_led_timer_mux);
                led_strip_1.running_led_offset_from_middle = 0;
                portEXIT_CRITICAL(&running_led_timer_mux);
                break;

            case 3:
                // led fade on + fade off mode
                timerAlarmWrite(fading_up_timer, 10000, true); // fade on + fade off should be more slowly than only fading on, so choose a bigger value for the alarm_value
                portENTER_CRITICAL(&fading_up_timer_mux);
                led_strip_1.led_current_brightness = 0;
                portEXIT_CRITICAL(&fading_up_timer_mux);
                led_strip_1.led_target_brightness_fade_on_fade_off = led_strip_1.led_target_brightness;
            
            default:
                break;
            }
    }

    void led_init_mode()
    {
        led_strip_1.led_target_red = 0;
        led_strip_1.led_target_green = 155;
        led_strip_1.led_target_blue = 155;
        led_strip_1.led_target_brightness = 25;
        led_strip_1.running_led_offset_from_middle = 0;
    }

    void led_standard_mode()
    {
        for(int i = 0; i < led_strip_1.num_leds; i++)
        {   
            leds[i] = CRGB(led_strip_1.led_target_red, led_strip_1.led_target_green, led_strip_1.led_target_blue);
        }
        led_strip_1.led_current_red = led_strip_1.led_target_red;
        led_strip_1.led_current_green = led_strip_1.led_target_green;
        led_strip_1.led_current_blue = led_strip_1.led_target_blue;
        led_strip_1.led_current_brightness = led_strip_1.led_target_brightness;
        FastLED.setBrightness(led_strip_1.led_target_brightness);
        FastLED.show();
    }

    void led_fade_on_mode()
    {
        // Mind that the variable led_current_brightness_ is increased/decreased in a seperate interrupt
        if (led_strip_1.led_target_brightness != led_strip_1.led_current_brightness ||
            led_strip_1.led_target_red != led_strip_1.led_current_red ||
            led_strip_1.led_target_green != led_strip_1.led_current_green ||
            led_strip_1.led_target_blue != led_strip_1.led_current_blue)
        {
            for(int i = 0; i < led_strip_1.num_leds; i++)
            {   
                leds[i] = CRGB(led_strip_1.led_target_red, led_strip_1.led_target_green, led_strip_1.led_target_blue);
            }
            led_strip_1.led_current_red = led_strip_1.led_target_red;
            led_strip_1.led_current_green = led_strip_1.led_target_green;
            led_strip_1.led_current_blue = led_strip_1.led_target_blue;
            FastLED.setBrightness(led_strip_1.led_current_brightness);
            FastLED.show();
        }
    }

    void led_closing_drawer_mode()
    {
        //TODO: If we have more then one led_strip, these paramters should become an argument of this function
        uint8_t middle_led = led_strip_1.middle_led;
        uint8_t num_of_led_shadows = led_strip_1.num_of_led_shadows;
        uint8_t running_led_offset_from_middle = led_strip_1.running_led_offset_from_middle;        

        if ((middle_led - running_led_offset_from_middle) >= 0 - num_of_led_shadows)
        {
            for(int i = 0; i < led_strip_1.num_leds; i++)
            {
                if ((i == (middle_led - running_led_offset_from_middle)) || (i == (middle_led + running_led_offset_from_middle)))
                {
                    leds[i] = CRGB(led_strip_1.led_target_red, led_strip_1.led_target_green, led_strip_1.led_target_blue);
                }
                // Create a shadow of running LED with less brightness
                else if ((running_led_offset_from_middle >= 1) && 
                        ((i == (middle_led - running_led_offset_from_middle + 1)) || (i == (middle_led + running_led_offset_from_middle - 1))))
                {
                    leds[i] = CRGB(led_strip_1.led_target_red/2, led_strip_1.led_target_green/2, led_strip_1.led_target_blue/2);
                }
                // Create a shadow of running LED with less brightness
                else if ((running_led_offset_from_middle >= 2) && 
                        ((i == (middle_led - running_led_offset_from_middle + 2)) || (i == (middle_led + running_led_offset_from_middle - 2))))
                {
                    leds[i] = CRGB(led_strip_1.led_target_red/3, led_strip_1.led_target_green/3, led_strip_1.led_target_blue/3);
                }
                // Create a shadow of running LED with less brightness
                else if ((running_led_offset_from_middle >= 3) && 
                        ((i == (middle_led - running_led_offset_from_middle + 3)) || (i == (middle_led + running_led_offset_from_middle - 3))))
                {
                    leds[i] = CRGB(led_strip_1.led_target_red/4, led_strip_1.led_target_green/4, led_strip_1.led_target_blue/4);
                }
                else
                {
                    leds[i] = CRGB(0, 0, 0);
                }
            }
            led_strip_1.led_current_red = led_strip_1.led_target_red;
            led_strip_1.led_current_green = led_strip_1.led_target_green;
            led_strip_1.led_current_blue = led_strip_1.led_target_blue;
            led_strip_1.led_current_brightness = led_strip_1.led_target_brightness;
            FastLED.setBrightness(led_strip_1.led_target_brightness);
            FastLED.show();
        }

        if ((middle_led - running_led_offset_from_middle) < 0 - num_of_led_shadows)
        {
            for(int i = 0; i < led_strip_1.num_leds; i++)
            {
                leds[i] = CRGB(0, 0, 0);
            }
            FastLED.setBrightness(0);
            FastLED.show();
        }
    }

    void led_fade_on_fade_off_mode()
    {
        // Mind that the variable led_current_brightness is increased/decreased in a seperate interrupt
        if (led_strip_1.led_target_brightness != led_strip_1.led_current_brightness ||
            led_strip_1.led_target_red != led_strip_1.led_current_red ||
            led_strip_1.led_target_green != led_strip_1.led_current_green ||
            led_strip_1.led_target_blue != led_strip_1.led_current_blue)
        {
            for(int i = 0; i < led_strip_1.num_leds; i++)
            {   
                leds[i] = CRGB(led_strip_1.led_target_red, led_strip_1.led_target_green, led_strip_1.led_target_blue);
            }
            led_strip_1.led_current_red = led_strip_1.led_target_red;
            led_strip_1.led_current_green = led_strip_1.led_target_green;
            led_strip_1.led_current_blue = led_strip_1.led_target_blue;
            FastLED.setBrightness(led_strip_1.led_current_brightness);
            FastLED.show();
        }

        // Mind that the variable led_current_brightness is increased/decreased in a seperate interrupt
        if (led_strip_1.led_current_brightness == 0)
        {
            led_strip_1.led_target_brightness = led_strip_1.led_target_brightness_fade_on_fade_off;
        }
        else if (led_strip_1.led_current_brightness == led_strip_1.led_target_brightness_fade_on_fade_off)
        {
            led_strip_1.led_target_brightness = 0;
        }
    }

    static void IRAM_ATTR on_timer_for_fading()
    {
        if (led_strip_1.led_target_brightness > led_strip_1.led_current_brightness)
        {
            portENTER_CRITICAL_ISR(&fading_up_timer_mux);
            led_strip_1.led_current_brightness++;
            portEXIT_CRITICAL_ISR(&fading_up_timer_mux);
        }

        if (led_strip_1.led_target_brightness < led_strip_1.led_current_brightness)
        {
            portENTER_CRITICAL_ISR(&fading_up_timer_mux);
            led_strip_1.led_current_brightness--;
            portEXIT_CRITICAL_ISR(&fading_up_timer_mux);
        }

        if (led_strip_2.led_target_brightness > led_strip_2.led_current_brightness)
        {
            portENTER_CRITICAL_ISR(&fading_up_timer_mux);
            led_strip_2.led_current_brightness++;
            portEXIT_CRITICAL_ISR(&fading_up_timer_mux);
        }

        if (led_strip_2.led_target_brightness < led_strip_2.led_current_brightness)
        {
            portENTER_CRITICAL_ISR(&fading_up_timer_mux);
            led_strip_2.led_current_brightness--;
            portEXIT_CRITICAL_ISR(&fading_up_timer_mux);
        }
    }

    static void IRAM_ATTR on_timer_for_running_led()
    {
        if ((led_strip_1.middle_led - led_strip_1.running_led_offset_from_middle) >= 0 - led_strip_1.num_of_led_shadows)
        {
            portENTER_CRITICAL_ISR(&running_led_timer_mux);
            led_strip_1.running_led_offset_from_middle++;
            portEXIT_CRITICAL_ISR(&running_led_timer_mux);
        }

        if ((led_strip_2.middle_led - led_strip_2.running_led_offset_from_middle) >= 0 - led_strip_2.num_of_led_shadows)
        {
            portENTER_CRITICAL_ISR(&running_led_timer_mux);
            led_strip_2.running_led_offset_from_middle++;
            portEXIT_CRITICAL_ISR(&running_led_timer_mux);
        }
    }

    void initialize_timer()
    {
        fading_up_timer_mux = portMUX_INITIALIZER_UNLOCKED;
        fading_up_timer = timerBegin(0, 80, true); // The base signal of the ESP32 has a frequency of 80Mhz -> prescaler 80 makes it 1Mhz
        timerAttachInterrupt(fading_up_timer, &on_timer_for_fading, true);
        timerAlarmWrite(fading_up_timer, 3000, true); // With the alarm_value of 3000 the interrupt will be triggert 333/s
        timerAlarmEnable(fading_up_timer);

        running_led_timer_mux = portMUX_INITIALIZER_UNLOCKED;
        running_led_timer = timerBegin(1, 80, true); // The base signal of the ESP32 has a frequency of 80Mhz -> prescaler 80 makes it 1Mhz
        timerAttachInterrupt(running_led_timer, &on_timer_for_running_led, true);
        timerAlarmWrite(running_led_timer, 50000, true); // 50000 is a good value. This defines how fast the LED will "run". Higher values will decrease the running speed.
        timerAlarmEnable(running_led_timer);
    }

    void handle_led_control(void)
    {
        switch (led_strip_1.led_mode)
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

    void initialize_led_strip(void)
    {
        std::memset(&led_strip_1, 0, sizeof(led_strip_1));
        std::memset(&led_strip_2, 0, sizeof(led_strip_2)); // currently we don't need led_strip_2, this is just here for example 
        led_strip_1.num_leds = 25;
        led_strip_1.middle_led = 13;
        led_strip_1.num_of_led_shadows = 3;

        FastLED.addLeds<NEOPIXEL,LED_PIXEL_PIN>(leds, NUM_OF_LEDS);

        led_init_mode();
        initialize_timer();
    }
    
} // namespace led_strip


#endif // LED_STRIP_HPP