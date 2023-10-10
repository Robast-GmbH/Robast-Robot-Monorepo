#ifndef DRYVE_D1_BRIDGE__SI_UNIT_FACTOR_HPP_
#define DRYVE_D1_BRIDGE__SI_UNIT_FACTOR_HPP_

namespace dryve_d1_bridge
{
  // Compare these values with hte values of page 164 of the dryve d1 manual:
  // https://assets.ctfassets.net/oxcgtdo88e20/AQWcl1jIZGhasmYsUkMOW/54cc1412717a60cd5d4dcf19fa624257/DE_DAS_Handbuch_dryve_D1_DE_V3.0.1.pdf
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
