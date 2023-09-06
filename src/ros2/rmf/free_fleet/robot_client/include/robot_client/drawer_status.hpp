#ifndef RMF__ROBOT_CLIENT__DRAWER_STATUS_HPP_
#define RMF__ROBOT_CLIENT__DRAWER_STATUS_HPP_
#include <string>

namespace rmf_robot_client
{
  struct DrawerStatus {
    int Module_id;
    int drawer_id;
    bool is_opened;
    std::string locked_for;

    // Constructor for convenience
    DrawerStatus(int moduleId, int drawerId, bool opened, const std::string lockedFor)
        : Module_id(moduleId), drawer_id(drawerId), is_opened(opened), locked_for(lockedFor) {}
  };
}// namespace robot_client
#endif   // RMF__ROBOT_CLIENT__DRAWER_STATUS_HPP_
