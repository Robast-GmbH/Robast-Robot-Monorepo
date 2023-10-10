#ifndef DRYVE_D1_BRIDGE__SI_UNIT_FACTOR_HPP_
#define DRYVE_D1_BRIDGE__SI_UNIT_FACTOR_HPP_

namespace dryve_d1_bridge
{
  enum class si_unit_factor : unsigned char
  {
    TEN_TO_THE_POWER_OF_2 = 0x02,
    TEN_TO_THE_POWER_OF_1 = 0x01,
    TEN_TO_THE_POWER_OF_0 = 0x00,
    TEN_TO_THE_POWER_OF_MINUS_1 = 0xFF,
    TEN_TO_THE_POWER_OF_MINUS_2 = 0xFE,
    TEN_TO_THE_POWER_OF_MINUS_3 = 0xFD,
    TEN_TO_THE_POWER_OF_MINUS_4 = 0xFC,
    TEN_TO_THE_POWER_OF_MINUS_5 = 0xFB,
    TEN_TO_THE_POWER_OF_MINUS_6 = 0xFA,
  };

}   // namespace dryve_d1_bridge

#endif   // DRYVE_D1_BRIDGE__SI_UNIT_FACTOR_HPP_
