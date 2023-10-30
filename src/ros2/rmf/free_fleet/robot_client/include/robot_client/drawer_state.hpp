#ifndef ROBOT_CLIENT__DRAWER_STATE_HPP_
#define ROBOT_CLIENT__DRAWER_STATE_HPP_
#include <string>
#include <vector>

#include "drawer_ref.hpp"

namespace rmf_robot_client
{
  struct DrawerState
  {
    DrawerRef drawer_ref;
    bool is_e_drawer;
    bool is_open;
    std::vector<u_int16_t> authorised_users;

    DrawerState(DrawerRef drawer_ref = DrawerRef(0, 0),
                bool e_drawer = false,
                bool open = false,
                std::vector<u_int16_t> authorisedUsers = std::vector<u_int16_t>())
        : drawer_ref(drawer_ref), is_e_drawer(e_drawer), is_open(open), authorised_users(authorisedUsers)
    {
    }

    bool is_set()
    {
      return !(drawer_ref.drawer_id == 0 && drawer_ref.module_id == 0);
    }
  };
}   // namespace rmf_robot_client
#endif   // ROBOT_CLIENT__DRAWER_STATE_HPP_
