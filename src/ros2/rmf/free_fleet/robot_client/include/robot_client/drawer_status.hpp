#ifndef RMF__ROBOT_CLIENT__DRAWER_STATUS_HPP_
#define RMF__ROBOT_CLIENT__DRAWER_STATUS_HPP_
#include <string>

namespace rmf_robot_client
{
  struct DrawerState {
    int module_id;
    int drawer_id;
    bool is_opened;
    std::vector<u_int16_t> authorised_users;

    // Constructor for convenience
    DrawerState(int moduleId, int drawerId, bool opened, std::vector<u_int16_t> authorisedUsers)
        : module_id(moduleId), drawer_id(drawerId), is_opened(opened), authorised_users(authorisedUsers) {}
  };
}// namespace robot_client
#endif   // RMF__ROBOT_CLIENT__DRAWER_STATUS_HPP_
