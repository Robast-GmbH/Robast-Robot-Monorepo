add_library(forward_robot_trajectory_plugin SHARED
  src/forward_robot_trajectory.cpp
)
set_target_properties(forward_robot_trajectory_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(forward_robot_trajectory_plugin ${THIS_PACKAGE_INCLUDE_DEPENDS})
