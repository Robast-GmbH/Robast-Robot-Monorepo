cmake_minimum_required(VERSION 3.8)
project(drawer_bridge_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(communication_interfaces REQUIRED)

include_directories(include)

set(executable_drawer_bridge_simulation drawer_bridge_simulation)

add_executable(${executable_drawer_bridge_simulation} src/drawer_bridge_simulation_main.cpp)

set(library_drawer_bridge_simulation ${executable_drawer_bridge_simulation}_core)

add_library(${library_drawer_bridge_simulation} SHARED
  src/drawer_bridge_simulation.cpp
)

set(dependencies
  rclcpp
  rclcpp_action
  control_msgs
  communication_interfaces
)

ament_target_dependencies(${executable_drawer_bridge_simulation}
  ${dependencies}
)
target_link_libraries(${executable_drawer_bridge_simulation} ${library_drawer_bridge_simulation})

ament_target_dependencies(${library_drawer_bridge_simulation}
  ${dependencies}
)

if(BUILD_TESTING)
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8()
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
  find_package(ament_cmake_pep257 REQUIRED)
  ament_pep257()
  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck()
endif()

install(TARGETS
  ${executable_drawer_bridge_simulation}
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
