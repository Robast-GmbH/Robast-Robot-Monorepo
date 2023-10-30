#ifndef DRYVE_D1_BRIDGE__D1_CONFIGS_HPP_
#define DRYVE_D1_BRIDGE__D1_CONFIGS_HPP_

#include <string>

namespace dryve_d1_bridge
{
  // this is actually the IP of the robot, whose router is forwarding this to the dryve d1 motor controls
  // we are interfacing with the different motor controls by targeting different ports (but using the same ip)
  // To run this on the robot use: 192.168.0.1
  // To run this from another pc in the robast network in the office use: 10.10.23.7
  const std::string DRYVE_D1_IP_ADDRESS = " 192.168.0.1";
  // these are the ports we configured in the router that are forwarded to port 502 (default modbus port) of the dryve
  // d1
  const int PORT_X_AXIS = 3503;
  const int PORT_Y_AXIS = 3502;

  const double D1_DEFAULT_VELOCITY = 1;
  const double D1_DEFAULT_ACCELERATION = 1;
  const double D1_DEFAULT_DECELERATION = 1;

  // Please mind: Actually you could get this value from the d1_dryve via get_si_unit_factor(), but I get stupid
  // values from it
  const double SI_UNIT_FACTOR = 100000;
  const double VELOCITY = 10;
  const double ACCELERATION = 10;
  const double DECELERATION = 10;
  // The value of the velocity scaling was obtained empirically. Instead of scaling the velocity with a factor like this
  // you could alternatively make the gains of the joint_trajectory_controller extremely high
  // TODO@Jacob: Keep checking if this is really the best way to handle this. I am not 100% sure about this
  const double VELOCITY_SCALING = 100;
}   // namespace dryve_d1_bridge
#endif   // DRYVE_D1_BRIDGE__D1_HPP_