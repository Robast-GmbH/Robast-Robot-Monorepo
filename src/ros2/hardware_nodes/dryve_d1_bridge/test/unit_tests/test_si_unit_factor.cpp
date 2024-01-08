#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>

#include "dryve_d1_bridge/si_unit_factor.hpp"
using namespace dryve_d1_bridge;
TEST_CASE("Testing map_si_unit_factor_double_to_char function", "[si_unit_factor]")
{
  SECTION("Testing with prismatic joint")
  {
    REQUIRE(map_si_unit_factor_double_to_char(0.00001, true) == si_unit_factor::TEN_TO_THE_POWER_OF_2);
    REQUIRE(map_si_unit_factor_double_to_char(0.0001, true) == si_unit_factor::TEN_TO_THE_POWER_OF_1);
    REQUIRE(map_si_unit_factor_double_to_char(0.001, true) == si_unit_factor::TEN_TO_THE_POWER_OF_0);
    REQUIRE(map_si_unit_factor_double_to_char(0.01, true) == si_unit_factor::TEN_TO_THE_POWER_OF_MINUS_1);
    REQUIRE(map_si_unit_factor_double_to_char(0.1, true) == si_unit_factor::TEN_TO_THE_POWER_OF_MINUS_2);
    REQUIRE(map_si_unit_factor_double_to_char(1.0, true) == si_unit_factor::TEN_TO_THE_POWER_OF_MINUS_3);
    REQUIRE(map_si_unit_factor_double_to_char(10.0, true) == si_unit_factor::TEN_TO_THE_POWER_OF_MINUS_4);
    REQUIRE(map_si_unit_factor_double_to_char(100.0, true) == si_unit_factor::TEN_TO_THE_POWER_OF_MINUS_5);
    REQUIRE(map_si_unit_factor_double_to_char(1000.0, true) == si_unit_factor::TEN_TO_THE_POWER_OF_MINUS_6);
    REQUIRE(map_si_unit_factor_double_to_char(9999.0, true) ==
            si_unit_factor::TEN_TO_THE_POWER_OF_MINUS_5);   // Default case
  }

  SECTION("Testing with non-prismatic joint")
  {
    REQUIRE(map_si_unit_factor_double_to_char(0.01, false) == si_unit_factor::TEN_TO_THE_POWER_OF_2);
    REQUIRE(map_si_unit_factor_double_to_char(0.1, false) == si_unit_factor::TEN_TO_THE_POWER_OF_1);
    REQUIRE(map_si_unit_factor_double_to_char(1.0, false) == si_unit_factor::TEN_TO_THE_POWER_OF_0);
    REQUIRE(map_si_unit_factor_double_to_char(10.0, false) == si_unit_factor::TEN_TO_THE_POWER_OF_MINUS_1);
    REQUIRE(map_si_unit_factor_double_to_char(100.0, false) == si_unit_factor::TEN_TO_THE_POWER_OF_MINUS_2);
    REQUIRE(map_si_unit_factor_double_to_char(1000.0, false) == si_unit_factor::TEN_TO_THE_POWER_OF_MINUS_3);
    REQUIRE(map_si_unit_factor_double_to_char(10000.0, false) == si_unit_factor::TEN_TO_THE_POWER_OF_MINUS_4);
    REQUIRE(map_si_unit_factor_double_to_char(100000.0, false) == si_unit_factor::TEN_TO_THE_POWER_OF_MINUS_5);
    REQUIRE(map_si_unit_factor_double_to_char(1000000.0, false) == si_unit_factor::TEN_TO_THE_POWER_OF_MINUS_6);
    REQUIRE(map_si_unit_factor_double_to_char(9999999.0, false) ==
            si_unit_factor::TEN_TO_THE_POWER_OF_MINUS_2);   // Default case
  }
}