#if !defined(LED_STRIP_HPP)
#define LED_STRIP_HPP

#include <Arduino.h>
#include <FastLED.h>

#include "pinout_defines.h"
#include "can/can_db.hpp"
#include "can/can_helper.h"

namespace led_strip
{
    class LedStrip
    {
        public:
            LedStrip(uint8_t num_leds, uint8_t middle_led, uint8_t num_of_led_shadows, uint8_t class_instance_id)
            {
                this->num_leds_ = num_leds; 
                this->middle_led_ = middle_led;
                this->num_of_led_shadows_ = num_of_led_shadows;
                this->class_instance_id_ = class_instance_id;
            }

            void initialize_led_strip(void)
            {
                FastLED.addLeds<NEOPIXEL,LED_PIXEL_PIN>(this->leds_, this->num_leds_ );
                this->led_current_brightness_ = 0;
                this->led_target_brightness_ = 0;
                this->led_init_mode();
                this->initialize_timer();
            }

            void handle_led_control(void)
            {
            switch (this->led_mode_)
                {
                case 0:
                    this->led_standard_mode();
                    break;

                case 1:
                    this->led_fade_on_mode();
                    break;

                case 2:
                    this->led_closing_drawer_mode();
                    break;

                case 3:
                    this->led_fade_on_fade_off_mode();
                    break;
                
                default:
                    this->led_standard_mode();
                    break;
                }
            }

            void select_led_strip_mode(robast_can_msgs::CanMessage can_message)
            {
                this->led_target_red_ = can_message.get_can_signals().at(CAN_SIGNAL_LED_RED).get_data();
                this->led_target_green_ = can_message.get_can_signals().at(CAN_SIGNAL_LED_GREEN).get_data();
                this->led_target_blue_ = can_message.get_can_signals().at(CAN_SIGNAL_LED_BLUE).get_data();
                this->led_target_brightness_ = can_message.get_can_signals().at(CAN_SIGNAL_LED_BRIGHTNESS).get_data();
                this->led_mode_ = can_message.get_can_signals().at(CAN_SIGNAL_LED_MODE).get_data();

                switch (this->led_mode_)
                    {
                    case 0:
                        // standard mode
                        break;

                    case 1:
                        // fade on mode
                        timerAlarmWrite(this->fading_up_timer_, 3000, true); // With the alarm_value of 3000 the interrupt will be triggert 333/s    
                        portENTER_CRITICAL(&this->fading_up_timer_mux_);
                        this->led_current_brightness_ = 0;
                        portEXIT_CRITICAL(&this->fading_up_timer_mux_);  
                        break;

                    case 2:
                        // led closing drawer mode
                        portENTER_CRITICAL(&this->running_led_timer_mux_);
                        this->running_led_offset_from_middle_ = 0;
                        portEXIT_CRITICAL(&this->running_led_timer_mux_);
                        break;

                    case 3:
                        // led fade on + fade off mode
                        timerAlarmWrite(this->fading_up_timer_, 10000, true); // fade on + fade off should be more slowly than only fading on, so choose a bigger value for the alarm_value
                        portENTER_CRITICAL(&this->fading_up_timer_mux_);
                        this->led_current_brightness_ = 0;
                        portEXIT_CRITICAL(&this->fading_up_timer_mux_);
                        this->led_target_brightness_fade_on_fade_off_ = this->led_target_brightness_;
                    
                    default:
                        break;
                    }
            }


        private:
            // To make the led strip brightness fade, we use ISRs. But these need to be static,
            // therefore the ISR does not know for which instance of a class it should change the member variables (e.g. led_current_brightness_).
            // To work around this, we need some glue routines to enable an interface between an ISR and an instance of a class like described here:
            // https://arduino.stackexchange.com/questions/89173/attach-the-arduino-isr-function-to-the-class-member
            static const uint8_t num_of_class_instances_ = 2;
            static LedStrip * instances [num_of_class_instances_];
            uint8_t class_instance_id_;

            uint8_t num_leds_; // number of LEDs for LED strip
            uint8_t middle_led_; // address of the middle LED, which is important for running LED mode
            uint8_t num_of_led_shadows_; // Number of "shadow" LEDs for running LED. At the moment you need to do a few more changes to increase the number of shadow LEDs, in the future it should only be this define

            CRGBArray<25> leds_; //TODO: FIX HARD CODED NUMBER
            volatile uint8_t led_current_brightness_;

            uint8_t led_target_brightness_;
            uint8_t led_target_red_;
            uint8_t led_target_green_;
            uint8_t led_target_blue_;
            
            uint8_t led_target_brightness_fade_on_fade_off_;
            uint8_t led_current_red_;
            uint8_t led_current_green_;
            uint8_t led_current_blue_;
            
            uint8_t led_mode_;

            hw_timer_t * fading_up_timer_ = NULL;
            portMUX_TYPE fading_up_timer_mux_ = portMUX_INITIALIZER_UNLOCKED;

            hw_timer_t * running_led_timer_ = NULL;
            portMUX_TYPE running_led_timer_mux_ = portMUX_INITIALIZER_UNLOCKED;

            volatile uint8_t running_led_offset_from_middle_ = 0; // this variable controls which LED is currently shining for the running LED mode

            void led_init_mode()
            {
                this->led_target_red_ = 0;
                this->led_target_green_ = 155;
                this->led_target_blue_ = 155;
                this->led_target_brightness_ = 25;
            }

            void led_standard_mode()
            {
                for(int i = 0; i < this->num_leds_; i++)
                {   
                    this->leds_[i] = CRGB(this->led_target_red_, this->led_target_green_, this->led_target_blue_);
                }
                this->led_current_red_ = this->led_target_red_;
                this->led_current_green_ = this->led_target_green_;
                this->led_current_blue_ = this->led_target_blue_;
                this->led_current_brightness_ = this->led_target_brightness_;
                FastLED.setBrightness(this->led_target_brightness_);
                FastLED.show();
            }

            void led_fade_on_mode()
            {
                // Mind that the variable led_current_brightness_ is increased/decreased in a seperate interrupt
                if (this->led_target_brightness_ != this->led_current_brightness_ ||
                    this->led_target_red_ != this->led_current_red_ ||
                    this->led_target_green_ != this->led_current_green_ ||
                    this->led_target_blue_ != this->led_current_blue_)
                {
                    for(int i = 0; i < this->num_leds_; i++)
                    {   
                        this->leds_[i] = CRGB(this->led_target_red_, this->led_target_green_, this->led_target_blue_);
                    }
                    this->led_current_red_ = this->led_target_red_;
                    this->led_current_green_ = this->led_target_green_;
                    this->led_current_blue_ = this->led_target_blue_;
                    FastLED.setBrightness(this->led_current_brightness_);
                    FastLED.show();
                }
            }

            void led_closing_drawer_mode()
            {
                if ((this->middle_led_ - this->running_led_offset_from_middle_) >= 0 - this->num_of_led_shadows_)
                {
                    for(int i = 0; i < this->num_leds_; i++)
                    {
                        if ((i == (this->middle_led_ - this->running_led_offset_from_middle_)) || (i == (this->middle_led_ + this->running_led_offset_from_middle_)))
                        {
                            this->leds_[i] = CRGB(this->led_target_red_, this->led_target_green_, this->led_target_blue_);
                        }
                        // Create a shadow of running LED with less brightness
                        else if ((this->running_led_offset_from_middle_ >= 1) && 
                                ((i == (this->middle_led_ - this->running_led_offset_from_middle_ + 1)) || (i == (this->middle_led_ + this->running_led_offset_from_middle_ - 1))))
                        {
                            this->leds_[i] = CRGB(this->led_target_red_/2, this->led_target_green_/2, this->led_target_blue_/2);
                        }
                        // Create a shadow of running LED with less brightness
                        else if ((this->running_led_offset_from_middle_ >= 2) && 
                                ((i == (this->middle_led_ - this->running_led_offset_from_middle_ + 2)) || (i == (this->middle_led_ + this->running_led_offset_from_middle_ - 2))))
                        {
                            this->leds_[i] = CRGB(this->led_target_red_/3, this->led_target_green_/3, this->led_target_blue_/3);
                        }
                        // Create a shadow of running LED with less brightness
                        else if ((this->running_led_offset_from_middle_ >= 3) && 
                                ((i == (this->middle_led_ - this->running_led_offset_from_middle_ + 3)) || (i == (this->middle_led_ + this->running_led_offset_from_middle_ - 3))))
                        {
                            this->leds_[i] = CRGB(this->led_target_red_/4, this->led_target_green_/4, this->led_target_blue_/4);
                        }
                        else
                        {
                            this->leds_[i] = CRGB(0, 0, 0);
                        }
                    }
                    this->led_target_blue_ = this->led_target_red_;
                    this->led_current_green_ = this->led_target_green_;
                    this->led_current_blue_ = this->led_target_blue_;
                    this->led_current_brightness_ = this->led_target_brightness_;
                    FastLED.setBrightness(this->led_target_brightness_);
                    FastLED.show();
                }

                if ((this->middle_led_ - this->running_led_offset_from_middle_) < 0 - this->num_of_led_shadows_)
                {
                    for(int i = 0; i < this->num_leds_; i++)
                    {
                        this->leds_[i] = CRGB(0, 0, 0);
                    }
                    FastLED.setBrightness(0);
                    FastLED.show();
                }
            }

            void led_fade_on_fade_off_mode()
            {
                // Mind that the variable led_current_brightness is increased/decreased in a seperate interrupt
                if (this->led_target_brightness_ != this->led_current_brightness_ ||
                    this->led_target_red_ != this->led_current_red_ ||
                    this->led_target_green_ != this->led_current_green_ ||
                    this->led_target_blue_ != this->led_current_blue_)
                {
                    for(int i = 0; i < this->num_leds_; i++)
                    {   
                        this->leds_[i] = CRGB(this->led_target_red_, this->led_target_green_, this->led_target_blue_);
                    }
                    this->led_current_red_ = this->led_target_red_;
                    this->led_current_green_ = this->led_target_green_;
                    this->led_current_blue_ = this->led_target_blue_;
                    FastLED.setBrightness(this->led_current_brightness_);
                    FastLED.show();
                }

                // Mind that the variable led_current_brightness is increased/decreased in a seperate interrupt
                if (this->led_current_brightness_ == 0)
                {
                    this->led_target_brightness_ = this->led_target_brightness_fade_on_fade_off_;
                }
                else if (this->led_current_brightness_ == this->led_target_brightness_fade_on_fade_off_)
                {
                    this->led_target_brightness_ = 0;
                }
            }

            static void IRAM_ATTR on_timer_for_fading_instance_1()
            {
                // if (LedStrip::instances[0]->led_target_brightness_ > LedStrip::instances[0]->led_current_brightness_)
                // {
                //     portENTER_CRITICAL_ISR(&LedStrip::instances[0]->fading_up_timer_mux_);
                //     // LedStrip::instances[0]->led_current_brightness_++;
                //     portEXIT_CRITICAL_ISR(&LedStrip::instances[0]->fading_up_timer_mux_);
                // }

                // if (LedStrip::instances[0]->led_target_brightness_ < LedStrip::instances[0]->led_current_brightness_)
                // {
                //     portENTER_CRITICAL_ISR(&LedStrip::instances[0]->fading_up_timer_mux_);
                //     // LedStrip::instances[0]->led_current_brightness_--;
                //     portEXIT_CRITICAL_ISR(&LedStrip::instances[0]->fading_up_timer_mux_);
                // }
            }

            static void IRAM_ATTR on_timer_for_running_led_instance_1()
            {
                // if ((LedStrip::instances[0]->middle_led_ - LedStrip::instances[0]->running_led_offset_from_middle_) >= 0 - LedStrip::instances[0]->num_of_led_shadows_)
                // {
                //     portENTER_CRITICAL_ISR(&LedStrip::instances[0]->running_led_timer_mux_);
                //     // LedStrip::instances[0]->running_led_offset_from_middle_++;
                //     portEXIT_CRITICAL_ISR(&LedStrip::instances[0]->running_led_timer_mux_);
                // }
            }

            static void IRAM_ATTR on_timer_for_fading_instance_2()
            {
                // if (LedStrip::instances[1]->led_target_brightness_ > LedStrip::instances[1]->led_current_brightness_)
                // {
                //     portENTER_CRITICAL_ISR(&LedStrip::instances[1]->fading_up_timer_mux_);
                //     // LedStrip::instances[1]->led_current_brightness_++;
                //     portEXIT_CRITICAL_ISR(&LedStrip::instances[1]->fading_up_timer_mux_);
                // }

                // if (LedStrip::instances[1]->led_target_brightness_ < LedStrip::instances[1]->led_current_brightness_)
                // {
                //     portENTER_CRITICAL_ISR(&LedStrip::instances[1]->fading_up_timer_mux_);
                //     // LedStrip::instances[1]->led_current_brightness_--;
                //     portEXIT_CRITICAL_ISR(&LedStrip::instances[1]->fading_up_timer_mux_);
                // }
            }

            static void IRAM_ATTR on_timer_for_running_led_instance_2()
            {
                // if ((LedStrip::instances[1]->middle_led_ - LedStrip::instances[1]->running_led_offset_from_middle_) >= 0 - LedStrip::instances[1]->num_of_led_shadows_)
                // {
                //     portENTER_CRITICAL_ISR(&LedStrip::instances[1]->running_led_timer_mux_);
                //     // LedStrip::instances[1]->running_led_offset_from_middle_++;
                //     portEXIT_CRITICAL_ISR(&LedStrip::instances[1]->running_led_timer_mux_);
                // }
            }

            void initialize_timer()
            {
                this->fading_up_timer_ = timerBegin(0, 80, true); // The base signal of the ESP32 has a frequency of 80Mhz -> prescaler 80 makes it 1Mhz
                timerAlarmWrite(this->fading_up_timer_, 3000, true); // With the alarm_value of 3000 the interrupt will be triggert 333/s
                timerAlarmEnable(this->fading_up_timer_);

                this->running_led_timer_ = timerBegin(1, 80, true); // The base signal of the ESP32 has a frequency of 80Mhz -> prescaler 80 makes it 1Mhz
                timerAlarmWrite(this->running_led_timer_, 50000, true); // 50000 is a good value. This defines how fast the LED will "run". Higher values will decrease the running speed.
                timerAlarmEnable(this->running_led_timer_);

                switch (this->class_instance_id_)
                {
                    case 1:
                        timerAttachInterrupt(this->fading_up_timer_, &on_timer_for_fading_instance_1, true);
                        timerAttachInterrupt(this->running_led_timer_, &on_timer_for_running_led_instance_1, true);
                        instances [0] = this;
                        break;

                    case 2:
                        timerAttachInterrupt(this->fading_up_timer_, &on_timer_for_fading_instance_2, true);
                        timerAttachInterrupt(this->running_led_timer_, &on_timer_for_running_led_instance_2, true);
                        instances [1] = this;
                }
            }
    };
    
} // namespace led_strip


#endif // LED_STRIP_HPP