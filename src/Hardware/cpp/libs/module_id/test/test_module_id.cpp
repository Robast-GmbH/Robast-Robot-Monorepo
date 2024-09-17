#include <catch2/catch_all.hpp>

#include "../include/module_id/module_id.hpp"

TEST_CASE("ModulePrefix values are correctly calculated", "[ModulePrefix]")
{
  using namespace module_id;

  REQUIRE(static_cast<uint32_t>(ModulePrefix::MANUAL_DRAWER_10x40) ==
          (module_category::TRANSPORTATION
             << (bit_length::MODULE_UNIQUE_ID_BIT_LENGTH + bit_length::SUBMODULE_TYPE_BIT_LENGTH) |
           module_type::transportation::MANUAL_DRAWER_10x40 << bit_length::MODULE_UNIQUE_ID_BIT_LENGTH));

  REQUIRE(static_cast<uint32_t>(ModulePrefix::MANUAL_DRAWER_20x40) ==
          (module_category::TRANSPORTATION
             << (bit_length::MODULE_UNIQUE_ID_BIT_LENGTH + bit_length::SUBMODULE_TYPE_BIT_LENGTH) |
           module_type::transportation::MANUAL_DRAWER_20x40 << bit_length::MODULE_UNIQUE_ID_BIT_LENGTH));

  REQUIRE(static_cast<uint32_t>(ModulePrefix::MANUAL_DRAWER_30x40) ==
          (module_category::TRANSPORTATION
             << (bit_length::MODULE_UNIQUE_ID_BIT_LENGTH + bit_length::SUBMODULE_TYPE_BIT_LENGTH) |
           module_type::transportation::MANUAL_DRAWER_30x40 << bit_length::MODULE_UNIQUE_ID_BIT_LENGTH));

  REQUIRE(static_cast<uint32_t>(ModulePrefix::E_DRAWER_10x40) ==
          (module_category::TRANSPORTATION
             << (bit_length::MODULE_UNIQUE_ID_BIT_LENGTH + bit_length::SUBMODULE_TYPE_BIT_LENGTH) |
           module_type::transportation::E_DRAWER_10x40 << bit_length::MODULE_UNIQUE_ID_BIT_LENGTH));

  REQUIRE(static_cast<uint32_t>(ModulePrefix::PARTIAL_DRAWER_10x40x8) ==
          (module_category::TRANSPORTATION
             << (bit_length::MODULE_UNIQUE_ID_BIT_LENGTH + bit_length::SUBMODULE_TYPE_BIT_LENGTH) |
           module_type::transportation::PARTIAL_DRAWER_10x40x8 << bit_length::MODULE_UNIQUE_ID_BIT_LENGTH));

  REQUIRE(static_cast<uint32_t>(ModulePrefix::DINNER_TRAYS) ==
          (module_category::TRANSPORTATION
             << (bit_length::MODULE_UNIQUE_ID_BIT_LENGTH + bit_length::SUBMODULE_TYPE_BIT_LENGTH) |
           module_type::transportation::DINNER_TRAYS << bit_length::MODULE_UNIQUE_ID_BIT_LENGTH));

  REQUIRE(static_cast<uint32_t>(ModulePrefix::SURGERY_TOOLS) ==
          (module_category::TRANSPORTATION
             << (bit_length::MODULE_UNIQUE_ID_BIT_LENGTH + bit_length::SUBMODULE_TYPE_BIT_LENGTH) |
           module_type::transportation::SURGERY_TOOLS << bit_length::MODULE_UNIQUE_ID_BIT_LENGTH));
}

TEST_CASE("generate_module_id function works correctly", "[generate_module_id]")
{
  using namespace module_id;

  REQUIRE(generate_module_id(ModulePrefix::MANUAL_DRAWER_10x40, 0x0001) == 0b000100010000000000000001);
  REQUIRE_FALSE(generate_module_id(ModulePrefix::MANUAL_DRAWER_10x40, 0x0001) == 0b000100010000000000000111);
  REQUIRE(generate_module_id(ModulePrefix::MANUAL_DRAWER_10x40, 0x0001) ==
          (static_cast<uint32_t>(ModulePrefix::MANUAL_DRAWER_10x40) | 0x0001));

  REQUIRE(generate_module_id(ModulePrefix::MANUAL_DRAWER_20x40, 0x0002) ==
          (static_cast<uint32_t>(ModulePrefix::MANUAL_DRAWER_20x40) | 0x0002));

  REQUIRE(generate_module_id(ModulePrefix::MANUAL_DRAWER_30x40, 0x0003) ==
          (static_cast<uint32_t>(ModulePrefix::MANUAL_DRAWER_30x40) | 0x0003));

  REQUIRE(generate_module_id(ModulePrefix::E_DRAWER_10x40, 0x0004) ==
          (static_cast<uint32_t>(ModulePrefix::E_DRAWER_10x40) | 0x0004));

  REQUIRE(generate_module_id(ModulePrefix::PARTIAL_DRAWER_10x40x8, 0x0005) ==
          (static_cast<uint32_t>(ModulePrefix::PARTIAL_DRAWER_10x40x8) | 0x0005));

  REQUIRE(generate_module_id(ModulePrefix::DINNER_TRAYS, 0x0006) ==
          (static_cast<uint32_t>(ModulePrefix::DINNER_TRAYS) | 0x0006));

  REQUIRE(generate_module_id(ModulePrefix::SURGERY_TOOLS, 0x0007) ==
          (static_cast<uint32_t>(ModulePrefix::SURGERY_TOOLS) | 0x0007));
}