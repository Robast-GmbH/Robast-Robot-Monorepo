#ifndef GPIO_GPIO_INFO_HPP
#define GPIO_GPIO_INFO_HPP

namespace gpio
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

}   // namespace gpio

#endif /* GPIO_GPIO_INFO_HPP */