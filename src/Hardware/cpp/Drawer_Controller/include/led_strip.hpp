#ifndef DRAWER_CONTROLLER_LED_STRIP_HPP
#define DRAWER_CONTROLLER_LED_STRIP_HPP

#include <Arduino.h>
#include <FastLED.h>

#include <optional>

#include "can/can_db.hpp"
#include "can/can_helper.h"
#include "pinout_defines.hpp"

namespace led_strip
{
/*********************************************************************************************************
 GLOBAL VARIABLES AND CONSTANTS
*********************************************************************************************************/
#define NUM_OF_LEDS                   18
#define MAX_NUM_OF_LED_MODES_IN_QUEUE 3

  struct LedState
  {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint8_t brightness;
  };

  struct LedAnimation
  {
    std::array<LedState, NUM_OF_LEDS> target_led_states;
    uint8_t fade_time_in_hundreds_of_ms;        // this will be set by the LED_HEADER CAN message
    uint16_t num_of_led_states_to_change = 0;   // this will be set by the LED_HEADER CAN message
    uint16_t start_index_led_states = 0;        // this will be set by the LED_HEADER CAN message

    // Default Constructor
    LedAnimation()
        : target_led_states({}),
          fade_time_in_hundreds_of_ms(0),
          num_of_led_states_to_change(0),
          start_index_led_states(0)   // Initialize members to default values
    {
    }

    // Parameterized Constructor
    LedAnimation(std::array<LedState, NUM_OF_LEDS> target_led_states_input,
                 const uint8_t fade_time_in_hundreds_of_ms_input,
                 const uint16_t num_of_led_states_to_change_input,
                 const uint16_t start_index_led_states_input)
        : target_led_states(target_led_states_input),
          fade_time_in_hundreds_of_ms(fade_time_in_hundreds_of_ms_input),
          num_of_led_states_to_change(num_of_led_states_to_change_input),
          start_index_led_states(start_index_led_states_input)
    {
    }
  };

  bool fading_in_progress;
  std::vector<LedState> current_led_states(NUM_OF_LEDS);    // used to apply current led state to led strip
  std::vector<LedState> starting_led_states(NUM_OF_LEDS);   // used for fading from starting to target led state

  LedAnimation target_led_animation;

  // this queue makes sure, that a requested led modes gets its time to finish the animation before the next led
  // mode is started
  // Please mind: Normaly you would not built a queue yourself and use xQueue from FreeRTOS or std::queue
  // But: std:queue did not work and just permanently threw expections that made the ESP32 reboot
  // To use xQueue you need to use xQueueCreate to create a queue and you need to specify the size, in bytes, required
  // to hold each item in the queue, which is not possible for the CanMessage class because it contains a vector of
  // CanSignals which has a different length depending on the CanMessage.
  // Therefore we built a queue with a vector, which should be fine in this case as the queue usually only contains
  // one or two feedback messages and is rarely used. Furthermore we try to keep it as efficient as possible and try
  // to follow what is explained here: https://youtu.be/fHNmRkzxHWs?t=2541
  std::vector<LedAnimation> led_animations_queue;
  uint8_t head_of_led_animations_queue;

  LedAnimation new_target_led_animation;

  volatile float fade_counter = 0;
  float max_fade_counter = 0;

  // TODO: This needs to go into the queue as well
  uint16_t num_of_led_states_to_change = 0;   // this will be set by the LED_HEADER CAN message
  uint16_t start_index_led_states = 0;        // this will be set by the LED_HEADER CAN message
  uint16_t current_index_led_states = 0;

  CRGBArray<NUM_OF_LEDS> leds;

  hw_timer_t *fading_timer = NULL;
  portMUX_TYPE fading_timer_mux;

  /*********************************************************************************************************
   FUNCTIONS
  *********************************************************************************************************/
  // TODO@Jacob: We have the same queue in can_utils. Make one class out of it
  void add_element_to_led_animation_queue(LedAnimation led_animation)
  {
    led_animations_queue.push_back(led_animation);
  }

  // TODO@Jacob: We have the same queue in can_utils. Make one class out of it
  std::optional<LedAnimation> get_element_from_led_animation_queue()
  {
    uint8_t num_of_msgs_in_queue = led_animations_queue.size();
    if (num_of_msgs_in_queue == 0)
    {
      return {};
    }
    // TODO@Jacob: This can probably be removed once we are 100% sure that the queue is not infinetly growing
    debug_printf(
      "get_element_from_led_animation_queue! num_of_msgs_in_queue = %d, led_animations_queue.capacity() = %d\n",
      num_of_msgs_in_queue,
      led_animations_queue.capacity());

    if (head_of_led_animations_queue == (num_of_msgs_in_queue - 1))
    {
      LedAnimation led_animation = led_animations_queue[head_of_led_animations_queue];
      led_animations_queue.clear();
      head_of_led_animations_queue = 0;
      return led_animation;
    }
    else
    {
      return led_animations_queue[head_of_led_animations_queue++];
    }
  }

  LedState create_led_state(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness)
  {
    LedState led_state = LedState{};
    led_state.red = red;
    led_state.green = green;
    led_state.red = red;
    led_state.brightness = brightness;
    return led_state;
  }

  void led_init_mode()
  {
    for (uint8_t i = 0; i < NUM_OF_LEDS; ++i)
    {
      new_target_led_animation.target_led_states[i] = create_led_state(0, 155, 155, 25);
    }
    num_of_led_states_to_change = NUM_OF_LEDS;   // TODO: Check if this correct
    start_index_led_states = 0;
    LedAnimation target_led_animation =
      LedAnimation(new_target_led_animation.target_led_states, 50, NUM_OF_LEDS, 0);   // TODO: remove magical number
    add_element_to_led_animation_queue(target_led_animation);
  }

  static void IRAM_ATTR on_timer_for_fading()
  {
    if (fade_counter < max_fade_counter)
    {
      portENTER_CRITICAL_ISR(&fading_timer_mux);
      ++fade_counter;
      portEXIT_CRITICAL_ISR(&fading_timer_mux);
    }
  }

  void initialize_timer()
  {
    fading_timer_mux = portMUX_INITIALIZER_UNLOCKED;
    fading_timer =
      timerBegin(0, 80, true);   // The base signal of the ESP32 has a frequency of 80Mhz -> prescaler 80 makes it 1Mhz
    timerAttachInterrupt(fading_timer, &on_timer_for_fading, true);

    // The minimal fade time we can set is 100ms. To be able to display this fade time in 256 (8 bit) brightness steps
    // we need the timer to trigger 256 times within 100ms.
    // Therefore we need a period of T = 100ms * 1/256 = 0.0003906250s
    // The period is determined by T = alarm_value / f
    // Therefore alarm_value = T * f = 0.0003906250s * 1Mhz = 390.625
    // Multiplying this by 8 gives us 6250
    timerAlarmWrite(fading_timer, 6250, true);
    timerAlarmEnable(fading_timer);
  }

  void set_max_counter_value(uint8_t new_fade_time_in_hundreds_of_ms)
  {
    // check comment in initialize_timer() function for derivation of this calculation
    max_fade_counter = new_fade_time_in_hundreds_of_ms * (UINT8_MAX / 8);
  }

  void init_fading(uint8_t new_fade_time_in_hundreds_of_ms)
  {
    debug_printf("Init fading with new_fade_time_in_hundreds_of_ms = %d \n", new_fade_time_in_hundreds_of_ms);
    fading_in_progress = true;
    set_max_counter_value(new_fade_time_in_hundreds_of_ms);
    initialize_timer();
    portENTER_CRITICAL_ISR(&fading_timer_mux);
    fade_counter = 0;
    portEXIT_CRITICAL_ISR(&fading_timer_mux);
  }

  void apply_led_states_to_led_strip()
  {
    debug_printf("apply_led_states_to_led_strip with brightness = %d \n", current_led_states[0].brightness);
    FastLED.setBrightness(255);   // individual led brightness will be scaled down later in the for loop
    for (uint8_t i = start_index_led_states; i < num_of_led_states_to_change; ++i)
    {
      uint8_t red = current_led_states[i].red;
      uint8_t green = current_led_states[i].green;
      uint8_t blue = current_led_states[i].blue;
      uint8_t brightness = current_led_states[i].brightness;
      leds[i].setRGB(red, green, blue);
      leds[i].fadeToBlackBy(255 - brightness);
      // debug_printf(
      //   "apply_led_states_to_led_strip with red: %d green: %d blue: %d brightness: %d\n", red, green, blue,
      //   brightness);
    }
    FastLED.show();
  }

  float linear_interpolation(float a, float b, float t)
  {
    return a + t * (b - a);
  }

  void set_current_led_states_to_target_led_states()
  {
    debug_println("Setting current led states to target led states...");
    timerAlarmDisable(fading_timer);   // Disable the timer alarm
    timerEnd(fading_timer);            // Stop and free timer
    fading_in_progress = false;
    for (uint16_t i = 0; i < NUM_OF_LEDS; ++i)
    {
      current_led_states[i].red = target_led_animation.target_led_states[i].red;
      current_led_states[i].green = target_led_animation.target_led_states[i].green;
      current_led_states[i].blue = target_led_animation.target_led_states[i].blue;
      current_led_states[i].brightness = target_led_animation.target_led_states[i].brightness;
    }
  }

  void handle_fading(void)
  {
    if (max_fade_counter == 0)
    {
      set_current_led_states_to_target_led_states();
    }
    else
    {
      // debug_printf("fade_counter = %f, max_fade_counter = %f\n", fade_counter, max_fade_counter);
      const float progress = fade_counter / max_fade_counter;
      // debug_printf("Fading progress %f\n", progress);
      if (progress < 1.0)
      {
        for (uint16_t i = 0; i < NUM_OF_LEDS; ++i)
        {
          current_led_states[i].red =
            linear_interpolation(starting_led_states[i].red, target_led_animation.target_led_states[i].red, progress);
          current_led_states[i].green = linear_interpolation(
            starting_led_states[i].green, target_led_animation.target_led_states[i].green, progress);
          current_led_states[i].blue =
            linear_interpolation(starting_led_states[i].blue, target_led_animation.target_led_states[i].blue, progress);
          current_led_states[i].brightness = linear_interpolation(
            starting_led_states[i].brightness, target_led_animation.target_led_states[i].brightness, progress);
        }
      }
      else
      {
        set_current_led_states_to_target_led_states();
      }
    }
  }

  void handle_led_control(void)
  {
    if (fading_in_progress)
    {
      handle_fading();
      apply_led_states_to_led_strip();
    }
    else
    {
      std::optional<LedAnimation> new_led_animation = get_element_from_led_animation_queue();
      if (new_led_animation.has_value())
      {
        target_led_animation = new_led_animation.value();
        init_fading(target_led_animation.fade_time_in_hundreds_of_ms);
      }
    }
  }

  void initialize_queue(void)
  {
    led_animations_queue.clear();
    head_of_led_animations_queue = 0;
  }

  void initialize_led_strip(void)
  {
    initialize_queue();

    FastLED.addLeds<NEOPIXEL, LED_PIXEL_PIN>(leds, NUM_OF_LEDS);

    led_init_mode();
    // initialize_timer();
  }

  void initialize_led_states(uint16_t num_of_led_states,
                             uint16_t start_index_led_states_input,
                             uint8_t fade_time_in_hundreds_of_ms_input)
  {
    num_of_led_states_to_change = num_of_led_states;
    start_index_led_states = start_index_led_states_input;
    current_index_led_states = start_index_led_states_input;
    new_target_led_animation.fade_time_in_hundreds_of_ms =
      fade_time_in_hundreds_of_ms_input;   // TODO@Jacob: set the timer according to that
  }

  void set_led_state(LedState state)
  {
    bool all_led_states_set = (num_of_led_states_to_change - start_index_led_states) == current_index_led_states;

    if (all_led_states_set)
    {
      LedAnimation led_animation = new_target_led_animation;
      add_element_to_led_animation_queue(led_animation);
    }
    else
    {
      new_target_led_animation.target_led_states[current_index_led_states] = state;
      current_index_led_states++;
    }
  }

}   // namespace led_strip

#endif   // DRAWER_CONTROLLER_LED_STRIP_HPP