#ifndef DRYVE_D1_BRIDGE__D1_CONFIGS_HPP_
#define DRYVE_D1_BRIDGE__D1_CONFIGS_HPP_

#include <string>

namespace dryve_d1_bridge
{
  constexpr double degree_to_rad(double degrees)
  {
    return degrees * M_PI / 180.0;
  }

  constexpr double mm_to_m(double mm)
  {
    return mm * 0.001;
  }

  // this is actually the IP of the robot, whose router is forwarding this to the dryve d1 motor controls
  // we are interfacing with the different motor controls by targeting different ports (but using the same ip)
  const std::string RB_THERON_IP_ADDRESS = "10.10.23.7";

  // These are the IP's the motor controller have within the network of the robot
  const std::string DRYVE_D1_IP_ADDRESS_Y_AXIS = "192.168.0.11";
  const std::string DRYVE_D1_IP_ADDRESS_X_AXIS = "192.168.0.53";
  const std::string DRYVE_D1_IP_ADDRESS_ROTATING_ARM = "192.168.0.12";
  // these are the ports we configured in the router that are forwarded to port 502 (default modbus port) of the d1
  const int RB_THERON_PORT_X_AXIS = 3503;
  const int RB_THERON_PORT_Y_AXIS = 3502;
  const int RB_THERON_PORT_ROTATING_ARM = 3504;

  const int MODBUS_TCP_PORT = 502;

  const double D1_DEFAULT_VELOCITY = 1;
  const double D1_DEFAULT_ACCELERATION = 1;
  const double D1_DEFAULT_DECELERATION = 1;

  const double VELOCITY = 10;
  const double ACCELERATION = 10;
  const double DECELERATION = 10;

} // namespace dryve_d1_bridge
#endif // DRYVE_D1_BRIDGE__D1_HPP_