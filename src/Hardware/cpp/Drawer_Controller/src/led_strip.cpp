#include "led/led_strip.hpp"

namespace drawer_controller
{
  LedStrip::LedStrip()
      : _starting_led_states(NUM_OF_LEDS),
        _current_led_states(NUM_OF_LEDS),
        _target_led_animation(std::vector<LedState>(NUM_OF_LEDS), 0, NUM_OF_LEDS, 0),
        _new_target_led_animation(std::vector<LedState>(NUM_OF_LEDS), 0, NUM_OF_LEDS, 0)
  {
    initialize_led_strip();
  }

  void LedStrip::add_element_to_led_animation_queue(LedAnimation led_animation)
  {
    _led_animations_queue.push_back(led_animation);
  }

  std::optional<LedAnimation> LedStrip::get_element_from_led_animation_queue()
  {
    uint8_t num_of_msgs_in_queue = _led_animations_queue.size();
    if (num_of_msgs_in_queue == 0)
    {
      return {};
    }
    // TODO@Jacob: This can probably be removed once we are 100% sure that the queue is not infinetly growing
    debug_printf(
      "get_element_from_led_animation_queue! num_of_msgs_in_queue = %d, _led_animations_queue.capacity() = %d\n",
      num_of_msgs_in_queue,
      _led_animations_queue.capacity());

    if (_head_of_led_animations_queue == (num_of_msgs_in_queue - 1))
    {
      LedAnimation led_animation = _led_animations_queue[_head_of_led_animations_queue];
      _led_animations_queue.clear();
      _head_of_led_animations_queue = 0;
      return led_animation;
    }
    else
    {
      return _led_animations_queue[_head_of_led_animations_queue++];
    }
  }

  void LedStrip::handle_led_control()
  {
    if (_is_fading_in_progress)
    {
      handle_fading();
      apply_led_states_to_led_strip();
    }
    else
    {
      std::optional<LedAnimation> new_led_animation = get_element_from_led_animation_queue();
      if (new_led_animation.has_value())
      {
        _target_led_animation = new_led_animation.value();
        init_fading(_target_led_animation.fade_time_in_hundreds_of_ms);
      }
    }
  }

  void LedStrip::led_init_mode()
  {
    _new_target_led_animation.target_led_states.clear();
    for (uint8_t i = 0; i < NUM_OF_LEDS; ++i)
    {
      _new_target_led_animation.target_led_states.push_back(
        LedState(LED_INIT_RED, LED_INIT_GREEN, LED_INIT_BLUE, LED_INIT_BRIGHTNESS));
    }
    LedAnimation initial_led_animation = LedAnimation(_new_target_led_animation.target_led_states,
                                                      LED_INIT_ANIMATION_FADE_TIME_IN_MS / 100,
                                                      NUM_OF_LEDS,
                                                      LED_INIT_ANIMATION_START_INDEX);
    add_element_to_led_animation_queue(initial_led_animation);
  }

  void LedStrip::init_fading(const uint8_t new_fade_time_in_hundreds_of_ms)
  {
    debug_printf("Init fading with new_fade_time_in_hundreds_of_ms = %d \n", new_fade_time_in_hundreds_of_ms);
    _is_fading_in_progress = true;
    timer::set_max_counter_value(new_fade_time_in_hundreds_of_ms, TIMER_FACTOR);
    timer::enable_timer();
  }

  void LedStrip::apply_led_states_to_led_strip()
  {
    FastLED.setBrightness(LED_MAX_BRIGHTNESS);   // individual led brightness will be scaled down in the for loop
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
      // debug_printf(
      //   "Setting led with index %d to red=%d, green=%d, blue=%d, brightness=%d!\n", i, red, green, blue, brightness);
    }
    FastLED.show();
  }

  float LedStrip::linear_interpolation(const float a, const float b, const float t)
  {
    return a + t * (b - a);
  }

  void LedStrip::set_current_led_states_to_target_led_states()
  {
    debug_println("Setting current led states to target led states...");
    timer::disable_timer();
    _is_fading_in_progress = false;
    for (uint16_t i = 0; i < NUM_OF_LEDS; ++i)
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

  void LedStrip::handle_fading(void)
  {
    if (timer::get_max_fade_counter_value == 0)
    {
      set_current_led_states_to_target_led_states();
    }
    else
    {
      const float progress = timer::get_fade_counter_value() / timer::get_max_fade_counter_value();
      if (progress < FULL_PROGRESS_LED_FADING)
      {
        for (uint16_t i = 0; i < NUM_OF_LEDS; ++i)
        {
          // TODO@Jacob: For the leds at the base it seems that all these interpolation computations disturb the change
          // TODO@Jacob: of the correct leds. At least when i comment this out the correct leds are changed.
          // TODO@Jacob: Find a way to not need the linear interpolation as this is computationally expensive
          // _current_led_states[i].red =
          //   linear_interpolation(_starting_led_states[i].red, _target_led_animation.target_led_states[i].red,
          //   progress);
          // _current_led_states[i].green = linear_interpolation(
          //   _starting_led_states[i].green, _target_led_animation.target_led_states[i].green, progress);
          // _current_led_states[i].blue = linear_interpolation(
          //   _starting_led_states[i].blue, _target_led_animation.target_led_states[i].blue, progress);
          _current_led_states[i].red = _target_led_animation.target_led_states[i].red;
          _current_led_states[i].green = _target_led_animation.target_led_states[i].green;
          _current_led_states[i].blue = _target_led_animation.target_led_states[i].blue;
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

  void LedStrip::initialize_queue()
  {
    _head_of_led_animations_queue = 0;
  }

  void LedStrip::initialize_led_strip()
  {
    initialize_queue();

    FastLED.addLeds<NEOPIXEL, LED_PIXEL_PIN>(_leds, NUM_OF_LEDS);

    led_init_mode();
    timer::initialize_timer();
  }

  void LedStrip::set_num_of_leds_to_change_to_value_within_bounds(const uint16_t num_of_led_states,
                                                                  const uint16_t start_index_led_states)
  {
    _new_target_led_animation.num_of_led_states_to_change =
      std::min(num_of_led_states, (uint16_t) (NUM_OF_LEDS - start_index_led_states));
  }

  void LedStrip::initialize_led_state_change(LedHeader led_header)
  {
    debug_printf(
      "Initialized led state change for %d num of leds with start index %d and fade_time_in_hundreds_of_ms %d!\n",
      led_header.num_of_led_states_to_change,
      led_header.start_index_of_leds_to_change,
      led_header.fade_time_in_hundreds_of_ms);

    set_num_of_leds_to_change_to_value_within_bounds(led_header.num_of_led_states_to_change,
                                                     led_header.start_index_of_leds_to_change);
    _new_target_led_animation.start_index_led_states = led_header.start_index_of_leds_to_change;
    _new_target_led_animation.fade_time_in_hundreds_of_ms = led_header.fade_time_in_hundreds_of_ms;
    _current_index_led_states = led_header.start_index_of_leds_to_change;
  }

  void LedStrip::set_led_state(LedState state)
  {
    bool all_leds_already_set = (_new_target_led_animation.num_of_led_states_to_change -
                                 _new_target_led_animation.start_index_led_states) <= _current_index_led_states;

    if (all_leds_already_set)
    {
      Serial.println("Warning! I received more led states then the header specified!");
      return;
    }
    debug_printf("Adding requested led state (red = %d, green = %d, blue = %d) change with index %d!\n",
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
      add_element_to_led_animation_queue(led_animation);
    }
  }

}   // namespace drawer_controller