#ifndef DRAWER_CONTROLLER_GPIO_INFO_HPP
#define DRAWER_CONTROLLER_GPIO_INFO_HPP

namespace drawer_controller
{
  constexpr bool IS_INPUT = true;
  constexpr bool IS_OUTPUT = false;

  struct GpioInfo
  {
    const uint8_t pin_number;
    const bool is_input;

    GpioInfo(const uint8_t pin_number_arg, const bool is_input_arg) : pin_number(pin_number_arg), is_input(is_input_arg)
    {
    }
  };

}   // namespace drawer_controller

#endif /* DRAWER_CONTROLLER_GPIO_INFO_HPP */