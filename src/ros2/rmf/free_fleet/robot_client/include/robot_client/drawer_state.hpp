#ifndef ROBOT_CLIENT__DRAWER_STATE_HPP_
#define ROBOT_CLIENT__DRAWER_STATE_HPP_
#include <string>
#include <vector>

namespace rmf_robot_client
{
struct DrawerState
{
  int module_id;
  int drawer_id;
  bool is_e_drawer;
  bool is_open;
  std::vector<u_int16_t> authorised_users;

  // Constructor for convenience
  DrawerState(
    int moduleId, int drawerId, bool e_drawer, bool open,
    std::vector<u_int16_t> authorisedUsers)
  : module_id(moduleId), drawer_id(drawerId), is_e_drawer(e_drawer), is_open(open),
    authorised_users(authorisedUsers) {}
};
}  // namespace rmf_robot_client
#endif  // ROBOT_CLIENT__DRAWER_STATE_HPP_
