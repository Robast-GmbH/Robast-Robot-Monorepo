#include "ultrasonic_hc_sr04/ultrasonic_hc_sr04.hpp"

namespace ultrasonic_sensor
{
  UltrasonicHCSR04::UltrasonicHCSR04() : Node("drawer_bridge")
  {
    setup_publishers();
  }

  void UltrasonicHCSR04::setup_publishers()
  {
    // TODO@Jacob: Add a publisher to publish the results of the ultrasonic sensor
  }

}   // namespace ultrasonic_sensor
