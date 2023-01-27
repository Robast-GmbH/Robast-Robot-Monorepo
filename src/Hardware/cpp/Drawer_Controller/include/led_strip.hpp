#if !defined(DRAWER_CONTROLLER_LED_STRIP_HPP)
#define DRAWER_CONTROLLER_LED_STRIP_HPP

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
    #define MIDDLE_LED 12
    #define NUM_OF_LED_SHADOWS 3
    #define MAX_NUM_OF_LED_MODES_IN_QUEUE 2

    enum LedMode
    {
        steady_light = 0,
        fade_up = 1,
        running_led_from_mid_to_outside = 2,
        slow_fade_up_fade_down = 3,
    };

    // this queue makes sure, that a requested led modes gets its time to finish the animation before the next led mode is started
    struct LedTargetSettings {
        uint8_t led_target_mode;
        uint8_t led_target_brightness;
        uint8_t led_target_red;
        uint8_t led_target_green;
        uint8_t led_target_blue;
    } led_strip_1_target_settings;

    struct LedStrip {
        const uint8_t num_leds; // number of LEDs for LED strip
        const uint8_t middle_led; // address of the middle LED, which is important for running LED mode
        const uint8_t num_of_led_shadows; // Number of "shadow" LEDs for running LED. At the moment you need to do a few more changes to increase the number of shadow LEDs, in the future it should only be this define

        volatile uint8_t led_current_brightness = 0;
        uint8_t led_current_red = 0;
        uint8_t led_current_green = 0;
        uint8_t led_current_blue = 0;
        uint8_t led_current_mode = 0;

        uint8_t led_target_brightness_fade_on_fade_off = 0;

        QueueHandle_t led_target_settings_queue;

        volatile uint8_t running_led_offset_from_middle = 0; // this variable controls which LED is currently shining for the running LED mode

        LedStrip(const uint8_t num_leds_input, const uint8_t middle_led_input, const uint8_t num_of_led_shadows_input) : num_leds(num_leds_input), middle_led(middle_led_input), num_of_led_shadows(num_of_led_shadows_input){}
    };

    LedStrip led_strip_1 = LedStrip(NUM_OF_LEDS, MIDDLE_LED, NUM_OF_LED_SHADOWS);
    LedStrip led_strip_2 = LedStrip(NUM_OF_LEDS, MIDDLE_LED, NUM_OF_LED_SHADOWS); // currently we don't need led_strip_2, this is just here for example 

    CRGBArray<NUM_OF_LEDS> leds;

    hw_timer_t * fading_up_timer = NULL;
    portMUX_TYPE fading_up_timer_mux;

    hw_timer_t * running_led_timer = NULL;
    portMUX_TYPE running_led_timer_mux;

    /*********************************************************************************************************
     FUNCTIONS
    *********************************************************************************************************/
    void select_led_strip_mode()
    {
        switch (led_strip_1_target_settings.led_target_mode)
            {
            case LedMode::steady_light:
                // standard mode
                led_strip_1.led_current_mode = LedMode::steady_light;
                break;

            case LedMode::fade_up:
                // fade up mode
                led_strip_1.led_current_mode = LedMode::fade_up;
                timerAlarmWrite(fading_up_timer, 3000, true); // With the alarm_value of 3000 the interrupt will be triggert 333/s    
                portENTER_CRITICAL(&fading_up_timer_mux);
                led_strip_1.led_current_brightness = 0;
                portEXIT_CRITICAL(&fading_up_timer_mux);  
                break;

            case LedMode::running_led_from_mid_to_outside:
                // led closing drawer mode
                led_strip_1.led_current_mode = LedMode::running_led_from_mid_to_outside;
                portENTER_CRITICAL(&running_led_timer_mux);
                led_strip_1.running_led_offset_from_middle = 0;
                portEXIT_CRITICAL(&running_led_timer_mux);
                break;

            case LedMode::slow_fade_up_fade_down:
                // led fade on + fade off mode
                led_strip_1.led_current_mode = LedMode::slow_fade_up_fade_down;
                timerAlarmWrite(fading_up_timer, 10000, true); // fade on + fade off should be more slowly than only fading on, so choose a bigger value for the alarm_value
                portENTER_CRITICAL(&fading_up_timer_mux);
                led_strip_1.led_current_brightness = 0;
                portEXIT_CRITICAL(&fading_up_timer_mux);
                led_strip_1.led_target_brightness_fade_on_fade_off = led_strip_1_target_settings.led_target_brightness;
            
            default:
                break;
            }
    }

    void add_led_strip_mode_to_queue(robast_can_msgs::CanMessage can_message)
    {
        auto led_target_settings = LedTargetSettings{};
        led_target_settings.led_target_red = can_message.get_can_signals().at(CAN_SIGNAL_LED_RED).get_data();
        led_target_settings.led_target_green = can_message.get_can_signals().at(CAN_SIGNAL_LED_GREEN).get_data();
        led_target_settings.led_target_blue = can_message.get_can_signals().at(CAN_SIGNAL_LED_BLUE).get_data();
        led_target_settings.led_target_brightness = can_message.get_can_signals().at(CAN_SIGNAL_LED_BRIGHTNESS).get_data();
        led_target_settings.led_target_mode = can_message.get_can_signals().at(CAN_SIGNAL_LED_MODE).get_data();

        Serial.print("Writing led strip mode to queue: "); //DEBUGGING
        Serial.println(led_target_settings.led_target_mode, DEC); //DEBUGGING

        // Adding led mode to queue
        xQueueSend(led_strip_1.led_target_settings_queue, &led_target_settings, 0); // setting the third argument to 0 makes sure, filling the queue wont block, if the queue is already full

        uint8_t led_modes_in_queue = uxQueueMessagesWaiting(led_strip_1.led_target_settings_queue);

        Serial.print("Led modes in queue: "); //DEBUGGING
        Serial.println(led_modes_in_queue, DEC); //DEBUGGING

        if (led_modes_in_queue == 1) {
             // Get the current target led settings from queue without removing it, because it will we removed after the animation is finished
            xQueuePeek(led_strip_1.led_target_settings_queue, &led_strip_1_target_settings, 0);
            select_led_strip_mode();
        }
    }

    void led_init_mode()
    {
        led_strip_1_target_settings.led_target_red = 0;
        led_strip_1_target_settings.led_target_green = 155;
        led_strip_1_target_settings.led_target_blue = 155;
        led_strip_1_target_settings.led_target_brightness = 25;
        led_strip_1.running_led_offset_from_middle = 0;
        led_strip_1.led_current_mode = 1;
    }

    void get_new_target_led_settings_from_queue()
    {
        xQueueReceive(led_strip_1.led_target_settings_queue, &led_strip_1_target_settings, 0); // Remove the last target led settings from queue
        uint8_t led_modes_in_queue = uxQueueMessagesWaiting(led_strip_1.led_target_settings_queue);

        // In case there is another target led setting waiting in the queue, get it and start it
        if (led_modes_in_queue >= 1) {
            xQueuePeek(led_strip_1.led_target_settings_queue, &led_strip_1_target_settings, 0); // Get the current target led settings from queue without removing it
            select_led_strip_mode();
        }
    }

    void led_standard_mode()
    {
        for(int i = 0; i < led_strip_1.num_leds; i++)
        {   
            leds[i] = CRGB(led_strip_1_target_settings.led_target_red, led_strip_1_target_settings.led_target_green, led_strip_1_target_settings.led_target_blue);
        }
        led_strip_1.led_current_red = led_strip_1_target_settings.led_target_red;
        led_strip_1.led_current_green = led_strip_1_target_settings.led_target_green;
        led_strip_1.led_current_blue = led_strip_1_target_settings.led_target_blue;
        led_strip_1.led_current_brightness = led_strip_1_target_settings.led_target_brightness;
        FastLED.setBrightness(led_strip_1.led_current_brightness);
        FastLED.show();

        get_new_target_led_settings_from_queue();

    }

    void led_fade_on_mode()
    {
        // Mind that the variable led_current_brightness_ is increased/decreased in a seperate interrupt
        if (led_strip_1_target_settings.led_target_brightness != led_strip_1.led_current_brightness ||
            led_strip_1_target_settings.led_target_red != led_strip_1.led_current_red ||
            led_strip_1_target_settings.led_target_green != led_strip_1.led_current_green ||
            led_strip_1_target_settings.led_target_blue != led_strip_1.led_current_blue)
        {
            for(int i = 0; i < led_strip_1.num_leds; i++)
            {   
                leds[i] = CRGB(led_strip_1_target_settings.led_target_red, led_strip_1_target_settings.led_target_green, led_strip_1_target_settings.led_target_blue);
            }
            led_strip_1.led_current_red = led_strip_1_target_settings.led_target_red;
            led_strip_1.led_current_green = led_strip_1_target_settings.led_target_green;
            led_strip_1.led_current_blue = led_strip_1_target_settings.led_target_blue;
            FastLED.setBrightness(led_strip_1.led_current_brightness);
            FastLED.show();
        }
        else if (led_strip_1_target_settings.led_target_brightness == led_strip_1.led_current_brightness)
        {
            get_new_target_led_settings_from_queue();
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
                    leds[i] = CRGB(led_strip_1_target_settings.led_target_red, led_strip_1_target_settings.led_target_green, led_strip_1_target_settings.led_target_blue);
                }
                // Create a shadow of running LED with less brightness
                else if ((running_led_offset_from_middle >= 1) && 
                        ((i == (middle_led - running_led_offset_from_middle + 1)) || (i == (middle_led + running_led_offset_from_middle - 1))))
                {
                    leds[i] = CRGB(led_strip_1_target_settings.led_target_red/2, led_strip_1_target_settings.led_target_green/2, led_strip_1_target_settings.led_target_blue/2);
                }
                // Create a shadow of running LED with less brightness
                else if ((running_led_offset_from_middle >= 2) && 
                        ((i == (middle_led - running_led_offset_from_middle + 2)) || (i == (middle_led + running_led_offset_from_middle - 2))))
                {
                    leds[i] = CRGB(led_strip_1_target_settings.led_target_red/3, led_strip_1_target_settings.led_target_green/3, led_strip_1_target_settings.led_target_blue/3);
                }
                // Create a shadow of running LED with less brightness
                else if ((running_led_offset_from_middle >= 3) && 
                        ((i == (middle_led - running_led_offset_from_middle + 3)) || (i == (middle_led + running_led_offset_from_middle - 3))))
                {
                    leds[i] = CRGB(led_strip_1_target_settings.led_target_red/4, led_strip_1_target_settings.led_target_green/4, led_strip_1_target_settings.led_target_blue/4);
                }
                else
                {
                    leds[i] = CRGB(0, 0, 0);
                }
            }
            led_strip_1.led_current_red = led_strip_1_target_settings.led_target_red;
            led_strip_1.led_current_green = led_strip_1_target_settings.led_target_green;
            led_strip_1.led_current_blue = led_strip_1_target_settings.led_target_blue;
            led_strip_1.led_current_brightness = led_strip_1_target_settings.led_target_brightness;
            FastLED.setBrightness(led_strip_1.led_current_brightness);
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

        if ((led_strip_1.middle_led - led_strip_1.running_led_offset_from_middle) == (0 - led_strip_1.num_of_led_shadows))
        {
            get_new_target_led_settings_from_queue();
        }
    }

    void led_fade_on_fade_off_mode()
    {
        // Mind that the variable led_current_brightness is increased/decreased in a seperate interrupt
        if (led_strip_1_target_settings.led_target_brightness != led_strip_1.led_current_brightness ||
            led_strip_1_target_settings.led_target_red != led_strip_1.led_current_red ||
            led_strip_1_target_settings.led_target_green != led_strip_1.led_current_green ||
            led_strip_1_target_settings.led_target_blue != led_strip_1.led_current_blue)
        {
            for(int i = 0; i < led_strip_1.num_leds; i++)
            {   
                leds[i] = CRGB(led_strip_1_target_settings.led_target_red, led_strip_1_target_settings.led_target_green, led_strip_1_target_settings.led_target_blue);
            }
            led_strip_1.led_current_red = led_strip_1_target_settings.led_target_red;
            led_strip_1.led_current_green = led_strip_1_target_settings.led_target_green;
            led_strip_1.led_current_blue = led_strip_1_target_settings.led_target_blue;
            FastLED.setBrightness(led_strip_1.led_current_brightness);
            FastLED.show();
        }

        // Mind that the variable led_current_brightness is increased/decreased in a seperate interrupt
        if (led_strip_1.led_current_brightness == 0)
        {
            led_strip_1_target_settings.led_target_brightness = led_strip_1.led_target_brightness_fade_on_fade_off;
        }
        else if (led_strip_1.led_current_brightness == led_strip_1.led_target_brightness_fade_on_fade_off)
        {
            led_strip_1_target_settings.led_target_brightness = 0;
        }
    }

    static void IRAM_ATTR on_timer_for_fading()
    {
        if (led_strip_1_target_settings.led_target_brightness > led_strip_1.led_current_brightness)
        {
            portENTER_CRITICAL_ISR(&fading_up_timer_mux);
            led_strip_1.led_current_brightness++;
            portEXIT_CRITICAL_ISR(&fading_up_timer_mux);
        }

        if (led_strip_1_target_settings.led_target_brightness < led_strip_1.led_current_brightness)
        {
            portENTER_CRITICAL_ISR(&fading_up_timer_mux);
            led_strip_1.led_current_brightness--;
            portEXIT_CRITICAL_ISR(&fading_up_timer_mux);
        }

        if (led_strip_1_target_settings.led_target_brightness > led_strip_2.led_current_brightness)
        {
            portENTER_CRITICAL_ISR(&fading_up_timer_mux);
            led_strip_2.led_current_brightness++;
            portEXIT_CRITICAL_ISR(&fading_up_timer_mux);
        }

        if (led_strip_1_target_settings.led_target_brightness < led_strip_2.led_current_brightness)
        {
            portENTER_CRITICAL_ISR(&fading_up_timer_mux);
            led_strip_2.led_current_brightness--;
            portEXIT_CRITICAL_ISR(&fading_up_timer_mux);
        }
    }

    static void IRAM_ATTR on_timer_for_running_led()
    {
        if ((led_strip_1.middle_led - led_strip_1.running_led_offset_from_middle) >= (0 - led_strip_1.num_of_led_shadows))
        {
            portENTER_CRITICAL_ISR(&running_led_timer_mux);
            led_strip_1.running_led_offset_from_middle++;
            portEXIT_CRITICAL_ISR(&running_led_timer_mux);
        }

        if ((led_strip_2.middle_led - led_strip_2.running_led_offset_from_middle) >= (0 - led_strip_2.num_of_led_shadows))
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
        switch (led_strip_1.led_current_mode)
            {
            case LedMode::steady_light:
                led_standard_mode();
                break;

            case LedMode::fade_up:
                led_fade_on_mode();
                break;

            case LedMode::running_led_from_mid_to_outside:
                led_closing_drawer_mode();
                break;

            case LedMode::slow_fade_up_fade_down:
                led_fade_on_fade_off_mode();
                break;
            
            default:
                led_standard_mode();
                break;
            }
    }

    void initialize_queue(void)
    {
        led_strip_1.led_target_settings_queue = xQueueCreate(MAX_NUM_OF_LED_MODES_IN_QUEUE, sizeof(LedTargetSettings));
        if(led_strip_1.led_target_settings_queue == NULL)
        {
            Serial.println("Error creating the led mode queue");
        }
    }

    void initialize_led_strip(void)
    {
        initialize_queue();

        FastLED.addLeds<NEOPIXEL,LED_PIXEL_PIN>(leds, NUM_OF_LEDS);

        led_init_mode();
        initialize_timer();
    }
    
} // namespace led_strip


#endif // DRAWER_CONTROLLER_LED_STRIP_HPP