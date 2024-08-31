#ifndef LED_LED_STRIP_HPP
#define LED_LED_STRIP_HPP

#include <Arduino.h>
#include <FastLED.h>

#include <algorithm>
#include <optional>

#include "can/can_db.hpp"
#include "can/can_helper.hpp"
#include "debug/debug.hpp"
#include "led/led_animation.hpp"
#include "led/led_header.hpp"
#include "led/led_state.hpp"
#include "timer/timer.hpp"
#include "utils/queue.hpp"

namespace led
{
  constexpr uint16_t LED_INIT_ANIMATION_FADE_TIME_IN_MS = 3000;
  constexpr uint8_t LED_INIT_ANIMATION_START_INDEX = 0;
  constexpr uint8_t LED_INIT_RED = 0;
  constexpr uint8_t LED_INIT_GREEN = 155;
  constexpr uint8_t LED_INIT_BLUE = 155;
  constexpr uint8_t LED_INIT_BRIGHTNESS = 25;
  constexpr uint8_t LED_MAX_BRIGHTNESS = 255;
  constexpr float FULL_PROGRESS_LED_FADING = 1.0;
  constexpr uint8_t MINIMAL_LOOP_TIME_IN_MS = 1;

  template <uint8_t led_pixel_pin, uint8_t num_of_leds>
  class LedStrip
  {
  public:
    LedStrip(const bool use_color_fading);

    void handle_led_control();

    void initialize_led_state_change(const LedHeader led_header);

    void set_led_state(LedState state);

   private:
    bool _is_fading_in_progress = false;
    std::vector<LedState> _starting_led_states;   // used for fading from starting to target led state
    std::vector<LedState> _current_led_states;    // used to apply current led state to led strip

    LedAnimation _target_led_animation;   // the current target led animation, which is applied withing fading time

    const std::unique_ptr<utils::Queue<LedAnimation>> _led_animations_queue = std::make_unique<utils::Queue<LedAnimation>>();

    LedAnimation _new_target_led_animation;   // the new target led animation that is successively filled by can msgs

    uint16_t _current_index_led_states = 0;

    CRGBArray<num_of_leds> _leds;

    unsigned long _previous_millis = 0; // makes sure that applying led animations is not done more then required

    const bool _use_color_fading;

    TickType_t _timestamp_last_led_show = 0;

    void led_init_mode();

    void init_fading(const uint8_t new_fade_time_in_hundreds_of_ms);

    void apply_led_states_to_led_strip();

    float linear_interpolation(const float a, const float b, const float t);

    void set_current_led_states_to_target_led_states();

    void handle_fading();

    void set_num_of_leds_to_change_to_value_within_bounds(const uint16_t num_of_led_states,
                                                          const uint16_t start_index_led_states);

    void initialize_led_strip();
  };

  /*********************************************************************************************************
   Implementations
   In C++ you need to include the implementation of the template class in the header file because the
   compiler needs to know the implementation of the template class when it is used in another file
  *********************************************************************************************************/

  template <uint8_t led_pixel_pin, uint8_t num_of_leds>
  LedStrip<led_pixel_pin, num_of_leds>::LedStrip(const bool use_color_fading)
      : _starting_led_states(num_of_leds),
        _current_led_states(num_of_leds),
        _target_led_animation(std::vector<LedState>(num_of_leds), 0, num_of_leds, 0),
        _new_target_led_animation(std::vector<LedState>(num_of_leds), 0, num_of_leds, 0),
        _use_color_fading(use_color_fading)
  {
    initialize_led_strip();
  }

  template <uint8_t led_pixel_pin, uint8_t num_of_leds>
  void LedStrip<led_pixel_pin, num_of_leds>::handle_led_control()
  {
    if (_is_fading_in_progress)
    {
      handle_fading();
      apply_led_states_to_led_strip();
    }
    else
    {
      std::optional<LedAnimation> new_led_animation = _led_animations_queue->dequeue();
      if (new_led_animation.has_value())
      {
        _target_led_animation = new_led_animation.value();
        init_fading(_target_led_animation.fade_time_in_hundreds_of_ms);
      }
    }
  }

  template <uint8_t led_pixel_pin, uint8_t num_of_leds>
  void LedStrip<led_pixel_pin, num_of_leds>::led_init_mode()
  {
    _new_target_led_animation.target_led_states.clear();
    for (uint8_t i = 0; i < num_of_leds; ++i)
    {
      _new_target_led_animation.target_led_states.push_back(
          LedState(LED_INIT_RED, LED_INIT_GREEN, LED_INIT_BLUE, LED_INIT_BRIGHTNESS));
    }
    LedAnimation initial_led_animation = LedAnimation(_new_target_led_animation.target_led_states,
                                                      LED_INIT_ANIMATION_FADE_TIME_IN_MS / 100,
                                                      num_of_leds,
                                                      LED_INIT_ANIMATION_START_INDEX);
    _led_animations_queue->enqueue(initial_led_animation);
  }

  template <uint8_t led_pixel_pin, uint8_t num_of_leds>
  void LedStrip<led_pixel_pin, num_of_leds>::init_fading(const uint8_t new_fade_time_in_hundreds_of_ms)
  {
    debug_printf("[LedStrip]: Init fading with new_fade_time_in_hundreds_of_ms = %d \n",
                 new_fade_time_in_hundreds_of_ms);
    _is_fading_in_progress = true;
    timer::set_max_counter_value(new_fade_time_in_hundreds_of_ms, timer::TIMER_FACTOR);
    timer::enable_timer();
  }

  template <uint8_t led_pixel_pin, uint8_t num_of_leds>
  void LedStrip<led_pixel_pin, num_of_leds>::apply_led_states_to_led_strip()
  {
    for (uint16_t i = _target_led_animation.start_index_led_states;
         i < _target_led_animation.num_of_led_states_to_change;
         ++i)
    {
      uint8_t red = _current_led_states[i].red;
      uint8_t green = _current_led_states[i].green;
      uint8_t blue = _current_led_states[i].blue;
      uint8_t brightness = _current_led_states[i].brightness;
      _leds[i].setRGB(red, green, blue);
      _leds[i].fadeToBlackBy(LED_MAX_BRIGHTNESS - brightness);
    }
    // Very important: We need enough time between the show() calls to let the LEDs show the color
    unsigned long loop_time = pdTICKS_TO_MS(xTaskGetTickCount() - _timestamp_last_led_show);
    if (loop_time < MINIMAL_LOOP_TIME_IN_MS)
    {
      vTaskDelay(pdMS_TO_TICKS(MINIMAL_LOOP_TIME_IN_MS - pdTICKS_TO_MS(loop_time)));
    }
    FastLED.show();
    _timestamp_last_led_show = xTaskGetTickCount();
  }

  template <uint8_t led_pixel_pin, uint8_t num_of_leds>
  float LedStrip<led_pixel_pin, num_of_leds>::linear_interpolation(const float a, const float b, const float t)
  {
    return a + t * (b - a);
  }

  template <uint8_t led_pixel_pin, uint8_t num_of_leds>
  void LedStrip<led_pixel_pin, num_of_leds>::set_current_led_states_to_target_led_states()
  {
    debug_println("[LedStrip]: Setting current led states to target led states...");
    timer::disable_timer();
    _is_fading_in_progress = false;
    for (uint16_t i = 0; i < num_of_leds; ++i)
    {
      _current_led_states[i].red = _target_led_animation.target_led_states[i].red;
      _current_led_states[i].green = _target_led_animation.target_led_states[i].green;
      _current_led_states[i].blue = _target_led_animation.target_led_states[i].blue;
      _current_led_states[i].brightness = _target_led_animation.target_led_states[i].brightness;

      // Save the led_states as starting_led_states for the next iteration
      _starting_led_states[i].red = _target_led_animation.target_led_states[i].red;
      _starting_led_states[i].green = _target_led_animation.target_led_states[i].green;
      _starting_led_states[i].blue = _target_led_animation.target_led_states[i].blue;
      _starting_led_states[i].brightness = _target_led_animation.target_led_states[i].brightness;
    }
  }

  template <uint8_t led_pixel_pin, uint8_t num_of_leds>
  void LedStrip<led_pixel_pin, num_of_leds>::handle_fading(void)
  {
    if (timer::get_max_fade_counter_value() == 0)
    {
      set_current_led_states_to_target_led_states();
    }
    else
    {
      const float progress = timer::get_fade_counter_value() / timer::get_max_fade_counter_value();
      if (progress < FULL_PROGRESS_LED_FADING)
      {
        for (uint16_t i = 0; i < num_of_leds; ++i)
        {
          // TODO@Jacob: Find a way to not need the linear interpolation as this is computationally expensive
          if (_use_color_fading)
          {
            _current_led_states[i].red =
                linear_interpolation(_starting_led_states[i].red, _target_led_animation.target_led_states[i].red,
                                     progress);
            _current_led_states[i].green = linear_interpolation(
                _starting_led_states[i].green, _target_led_animation.target_led_states[i].green, progress);
            _current_led_states[i].blue = linear_interpolation(
                _starting_led_states[i].blue, _target_led_animation.target_led_states[i].blue, progress);
          }
          else
          {
            if (_current_led_states[i].brightness < _target_led_animation.target_led_states[i].brightness)
            {
              _current_led_states[i].red = _target_led_animation.target_led_states[i].red;
              _current_led_states[i].green = _target_led_animation.target_led_states[i].green;
              _current_led_states[i].blue = _target_led_animation.target_led_states[i].blue;
            }
          }
          _current_led_states[i].brightness = linear_interpolation(
              _starting_led_states[i].brightness, _target_led_animation.target_led_states[i].brightness, progress);
        }
      }
      else
      {
        set_current_led_states_to_target_led_states();
      }
    }
  }

  template <uint8_t led_pixel_pin, uint8_t num_of_leds>
  void LedStrip<led_pixel_pin, num_of_leds>::initialize_led_strip()
  {
    FastLED.addLeds<NEOPIXEL, led_pixel_pin>(_leds, num_of_leds);

    led_init_mode();
    timer::initialize_timer();
  }

  template <uint8_t led_pixel_pin, uint8_t num_of_leds>
  void LedStrip<led_pixel_pin, num_of_leds>::set_num_of_leds_to_change_to_value_within_bounds(
    const uint16_t num_of_led_states, const uint16_t start_index_led_states)
  {
    _new_target_led_animation.num_of_led_states_to_change =
      std::min(num_of_led_states, (uint16_t) (num_of_leds - start_index_led_states));
  }

  template <uint8_t led_pixel_pin, uint8_t num_of_leds>
  void LedStrip<led_pixel_pin, num_of_leds>::initialize_led_state_change(LedHeader led_header)
  {
    debug_printf(
      "[LedStrip]: Initialized led state change for %d num of leds with start index %d and fade_time_in_hundreds_of_ms "
      "%d!\n",
      led_header.num_of_led_states_to_change,
      led_header.start_index_of_leds_to_change,
      led_header.fade_time_in_hundreds_of_ms);

    set_num_of_leds_to_change_to_value_within_bounds(led_header.num_of_led_states_to_change,
                                                     led_header.start_index_of_leds_to_change);
    _new_target_led_animation.start_index_led_states = led_header.start_index_of_leds_to_change;
    _new_target_led_animation.fade_time_in_hundreds_of_ms = led_header.fade_time_in_hundreds_of_ms;
    _current_index_led_states = led_header.start_index_of_leds_to_change;
  }

  template <uint8_t led_pixel_pin, uint8_t num_of_leds>
  void LedStrip<led_pixel_pin, num_of_leds>::set_led_state(LedState state)
  {
    bool all_leds_already_set = (_new_target_led_animation.num_of_led_states_to_change -
                                 _new_target_led_animation.start_index_led_states) <= _current_index_led_states;

    if (all_leds_already_set)
    {
      debug_println("Warning! I received more led states then the header specified!");
      return;
    }
    debug_printf("[LedStrip]: Adding requested led state (red = %d, green = %d, blue = %d) change with index %d!\n",
                 state.red,
                 state.green,
                 state.blue,
                 _current_index_led_states);
    _new_target_led_animation.target_led_states[_current_index_led_states] = state;
    ++_current_index_led_states;

    bool all_led_states_set = (_new_target_led_animation.num_of_led_states_to_change -
                               _new_target_led_animation.start_index_led_states) == _current_index_led_states;

    if (all_led_states_set)
    {
      LedAnimation led_animation = _new_target_led_animation;   // deep copy (see "=" operator definition in struct)
      _led_animations_queue->enqueue(led_animation);
    }
  }

}   // namespace led

#endif   // LED_LED_STRIP_HPP