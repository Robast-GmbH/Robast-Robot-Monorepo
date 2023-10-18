#ifndef DRYVE_D1_BRIDGE__D1_CONFIGS_HPP_
#define DRYVE_D1_BRIDGE__D1_CONFIGS_HPP_

#include <string>

namespace dryve_d1_bridge
{
  const std::string DRYVE_D1_IP_ADDRESS_X_AXIS = "10.10.23.7";
  const std::string DRYVE_D1_IP_ADDRESS_Y_AXIS = "10.10.23.7";
  // const int PORT = 502;
  const int PORT_X_AXIS = 3503;
  const int PORT_Y_AXIS = 3502;

  const double D1_DEFAULT_VELOCITY = 1;
  const double D1_DEFAULT_ACCELERATION = 1;
  const double D1_DEFAULT_DECELERATION = 1;

  // Please mind: Actually you could get this value from the d1_dryve via get_si_unit_factor(), but I get stupid
  // values from it
  const double X_AXIS_SI_UNIT_FACTOR = -100000;
  const double X_AXIS_VELOCITY = 10;
  const double X_AXIS_ACCELERATION = 10;
  const double X_AXIS_DECELERATION = 10;

  const double Y_AXIS_SI_UNIT_FACTOR = 100000;
  const double Y_AXIS_VELOCITY = 10;
  const double Y_AXIS_ACCELERATION = 10;
  const double Y_AXIS_DECELERATION = 10;
  // The value of the velocity scaling was obtained empirically. Instead of scaling the velocity with a factor like this
  // you could alternatively make the gains of the joint_trajectory_controller extremely high
  const double Y_AXIS_VELOCITY_SCALING = 100;
}   // namespace dryve_d1_bridge

#endif   // DRYVE_D1_BRIDGE__D1_HPP_