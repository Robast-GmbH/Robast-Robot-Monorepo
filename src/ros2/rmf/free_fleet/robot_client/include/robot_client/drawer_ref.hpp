#ifndef ROBOT_CLIENT__DRAWER_REF_HPP_
#define ROBOT_CLIENT__DRAWER_REF_HPP_
#include <string>

namespace rmf_robot_client
{
  struct DrawerRef
  {
    int module_id;
    int drawer_id;

    // Constructor for convenience
    DrawerRef(int moduleId, int drawerId) : module_id(moduleId), drawer_id(drawerId)
    {
    }

    bool operator==(const DrawerRef &other_drawer_ref)
    {
      return (module_id == other_drawer_ref.module_id && drawer_id == other_drawer_ref.drawer_id);
    }

    std::string get_ref_string()
    {
      return std::to_string(module_id) + "#" + std::to_string(drawer_id);
    }
  };
}   // namespace rmf_robot_client
#endif   // ROBOT_CLIENT__DRAWER_REF_HPP_
