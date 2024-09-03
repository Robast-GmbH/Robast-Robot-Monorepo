#ifndef MODULE_ID__MODULE_ID_HPP
#define MODULE_ID__MODULE_ID_HPP

#include <cstdint>

namespace module_id
{
  // Anonymous namespace for private constants and helper functions
  namespace
  {
    namespace bit_length
    {
      constexpr uint32_t MODULE_UNIQUE_ID_BIT_LENGTH = 16;
      constexpr uint32_t SUBMODULE_TYPE_BIT_LENGTH = 4;
      constexpr uint32_t MODULE_TYPE_BIT_LENGTH = 4;
    }   // namespace bit_length

    namespace module_type
    {
      constexpr uint32_t VISUALIZATION = 0b0000;
      constexpr uint32_t TRANSPORTATION = 0b0001;
    }   // namespace module_type

    namespace submodule_type
    {
      namespace visualization
      {
        constexpr uint32_t BASE_LED_CONTROLLER = 0b0000;
      }
      namespace transportation
      {
        constexpr uint32_t MANUAL_DRAWER_10x40 = 0b0001;
        constexpr uint32_t MANUAL_DRAWER_20x40 = 0b0010;
        constexpr uint32_t MANUAL_DRAWER_30x40 = 0b0011;
        constexpr uint32_t E_DRAWER_10x40 = 0b0100;
        constexpr uint32_t PARTIAL_DRAWER_10x40x8 = 0b0101;
        constexpr uint32_t DINNER_TRAYS = 0b0110;
        constexpr uint32_t SURGERY_TOOLS = 0b0111;
      }   // namespace transportation
    }   // namespace submodule_type
  }   // namespace

  enum class ModulePrefix : uint32_t
  {
    MANUAL_DRAWER_10x40 =
      module_type::TRANSPORTATION << (bit_length::MODULE_UNIQUE_ID_BIT_LENGTH + bit_length::SUBMODULE_TYPE_BIT_LENGTH) |
      submodule_type::transportation::MANUAL_DRAWER_10x40 << bit_length::MODULE_UNIQUE_ID_BIT_LENGTH,
    MANUAL_DRAWER_20x40 =
      module_type::TRANSPORTATION << (bit_length::MODULE_UNIQUE_ID_BIT_LENGTH + bit_length::SUBMODULE_TYPE_BIT_LENGTH) |
      submodule_type::transportation::MANUAL_DRAWER_20x40 << bit_length::MODULE_UNIQUE_ID_BIT_LENGTH,
    MANUAL_DRAWER_30x40 =
      module_type::TRANSPORTATION << (bit_length::MODULE_UNIQUE_ID_BIT_LENGTH + bit_length::SUBMODULE_TYPE_BIT_LENGTH) |
      submodule_type::transportation::MANUAL_DRAWER_30x40 << bit_length::MODULE_UNIQUE_ID_BIT_LENGTH,
    E_DRAWER_10x40 = module_type::TRANSPORTATION
                       << (bit_length::MODULE_UNIQUE_ID_BIT_LENGTH + bit_length::SUBMODULE_TYPE_BIT_LENGTH) |
                     submodule_type::transportation::E_DRAWER_10x40 << bit_length::MODULE_UNIQUE_ID_BIT_LENGTH,
    PARTIAL_DRAWER_10x40x8 =
      module_type::TRANSPORTATION << (bit_length::MODULE_UNIQUE_ID_BIT_LENGTH + bit_length::SUBMODULE_TYPE_BIT_LENGTH) |
      submodule_type::transportation::PARTIAL_DRAWER_10x40x8 << bit_length::MODULE_UNIQUE_ID_BIT_LENGTH,
    DINNER_TRAYS = module_type::TRANSPORTATION
                     << (bit_length::MODULE_UNIQUE_ID_BIT_LENGTH + bit_length::SUBMODULE_TYPE_BIT_LENGTH) |
                   submodule_type::transportation::DINNER_TRAYS << bit_length::MODULE_UNIQUE_ID_BIT_LENGTH,
    SURGERY_TOOLS = module_type::TRANSPORTATION
                      << (bit_length::MODULE_UNIQUE_ID_BIT_LENGTH + bit_length::SUBMODULE_TYPE_BIT_LENGTH) |
                    submodule_type::transportation::SURGERY_TOOLS << bit_length::MODULE_UNIQUE_ID_BIT_LENGTH
  };

  constexpr uint32_t generate_module_id(ModulePrefix module_prefix, uint32_t unique_id)
  {
    return static_cast<uint32_t>(module_prefix) | unique_id;
  }

}   // namespace module_id

#endif   // MODULE_ID__MODULE_ID_HPP