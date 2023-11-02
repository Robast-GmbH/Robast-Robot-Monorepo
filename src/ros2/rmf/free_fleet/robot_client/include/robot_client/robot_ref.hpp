#ifndef ROBOT_CLIENT__ROBOT_REF_HPP_
#define ROBOT_CLIENT__ROBOT_REF_HPP_
#include <string>

namespace rmf_robot_client
{
  struct RobotRef
  {
    std::string robot_name;
    std::string fleet_name;

    // Constructor for convenience
    RobotRef(std::string robotName = "", std::string fleetName = "") : robot_name(robotName), fleet_name(fleetName)
    {
    }
  };
}   // namespace rmf_robot_client
#endif   // ROBOT_CLIENT__ROBOT_REF_HPP_
