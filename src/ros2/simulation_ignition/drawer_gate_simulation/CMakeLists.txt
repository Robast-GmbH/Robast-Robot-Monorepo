cmake_minimum_required(VERSION 3.8)
project(drawer_gate_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(communication_interfaces REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

include_directories(include)

set(executable_drawer_gate_simulation drawer_gate_simulation)

add_executable(${executable_drawer_gate_simulation} src/drawer_gate_simulation_main.cpp)

set(library_drawer_gate_simulation ${executable_drawer_gate_simulation}_core)

add_library(${library_drawer_gate_simulation} SHARED
  src/drawer_gate_simulation.cpp
)

set(dependencies
  rclcpp
  moveit_ros_planning_interface
  communication_interfaces
)

ament_target_dependencies(${executable_drawer_gate_simulation}
  ${dependencies}
)
target_link_libraries(${executable_drawer_gate_simulation} ${library_drawer_gate_simulation})

ament_target_dependencies(${library_drawer_gate_simulation}
  ${dependencies}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  ${executable_drawer_gate_simulation}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
