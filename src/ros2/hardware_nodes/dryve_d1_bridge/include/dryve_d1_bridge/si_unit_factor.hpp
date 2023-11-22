#ifndef DRYVE_D1_BRIDGE__SI_UNIT_FACTOR_HPP_
#define DRYVE_D1_BRIDGE__SI_UNIT_FACTOR_HPP_

namespace dryve_d1_bridge
{
  // Compare these values with the values of page 164 of the dryve d1 manual:
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

  si_unit_factor map_si_unit_factor_double_to_char(double si_unit_factor, bool _is_prismatic_joint)
  {
    if (_is_prismatic_joint)
    {
      // switch case does not work for double, so we use if else here. If you know a better way, please let me know.
      if (si_unit_factor == 0.00001)
      {
        return si_unit_factor::TEN_TO_THE_POWER_OF_2;
      }
      else if (si_unit_factor == 0.0001)
      {
        return si_unit_factor::TEN_TO_THE_POWER_OF_1;
      }
      else if (si_unit_factor == 0.001)
      {
        return si_unit_factor::TEN_TO_THE_POWER_OF_0;
      }
      else if (si_unit_factor == 0.01)
      {
        return si_unit_factor::TEN_TO_THE_POWER_OF_MINUS_1;
      }
      else if (si_unit_factor == 0.1)
      {
        return si_unit_factor::TEN_TO_THE_POWER_OF_MINUS_2;
      }
      else if (si_unit_factor == 1.0)
      {
        return si_unit_factor::TEN_TO_THE_POWER_OF_MINUS_3;
      }
      else if (si_unit_factor == 10.0)
      {
        return si_unit_factor::TEN_TO_THE_POWER_OF_MINUS_4;
      }
      else if (si_unit_factor == 100.0)
      {
        return si_unit_factor::TEN_TO_THE_POWER_OF_MINUS_5;
      }
      else if (si_unit_factor == 1000.0)
      {
        return si_unit_factor::TEN_TO_THE_POWER_OF_MINUS_6;
      }
      else
      {
        return si_unit_factor::TEN_TO_THE_POWER_OF_MINUS_5;   // Default case for prismatic / linear joint
      }
    }
    else
    {
      if (si_unit_factor == 0.01)
      {
        return si_unit_factor::TEN_TO_THE_POWER_OF_2;
      }
      else if (si_unit_factor == 0.1)
      {
        return si_unit_factor::TEN_TO_THE_POWER_OF_1;
      }
      else if (si_unit_factor == 1.0)
      {
        return si_unit_factor::TEN_TO_THE_POWER_OF_0;
      }
      else if (si_unit_factor == 10.0)
      {
        return si_unit_factor::TEN_TO_THE_POWER_OF_MINUS_1;
      }
      else if (si_unit_factor == 100.0)
      {
        return si_unit_factor::TEN_TO_THE_POWER_OF_MINUS_2;
      }
      else if (si_unit_factor == 1000.0)
      {
        return si_unit_factor::TEN_TO_THE_POWER_OF_MINUS_3;
      }
      else if (si_unit_factor == 10000.0)
      {
        return si_unit_factor::TEN_TO_THE_POWER_OF_MINUS_4;
      }
      else if (si_unit_factor == 100000.0)
      {
        return si_unit_factor::TEN_TO_THE_POWER_OF_MINUS_5;
      }
      else if (si_unit_factor == 1000000.0)
      {
        return si_unit_factor::TEN_TO_THE_POWER_OF_MINUS_6;
      }
      else
      {
        return si_unit_factor::TEN_TO_THE_POWER_OF_MINUS_2;   // Default case for revolute / rotating joint
      }
    }
  }

}   // namespace dryve_d1_bridge

#endif   // DRYVE_D1_BRIDGE__SI_UNIT_FACTOR_HPP_
