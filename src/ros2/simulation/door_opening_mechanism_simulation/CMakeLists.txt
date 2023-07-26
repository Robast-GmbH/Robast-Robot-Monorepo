cmake_minimum_required(VERSION 3.8)
project(door_opening_mechanism_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(communication_interfaces REQUIRED)
find_package(depthai_ros_msgs REQUIRED)
find_package(graph_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(moveit_visual_tools REQUIRED)

include_directories(include)

set(executable_door_opening_mechanism_simulation door_opening_mechanism_simulation)
set(executable_door_opening_mechanism_simulation_send_fake_pose door_opening_mechanism_simulation_send_fake_pose)
set(executable_door_opening_mechanism_simulation_motion_plan_request door_opening_mechanism_simulation_motion_plan_request)

add_executable(${executable_door_opening_mechanism_simulation} src/door_opening_mechanism_simulation_main.cpp)
add_executable(${executable_door_opening_mechanism_simulation_send_fake_pose} src/door_opening_mechanism_simulation_send_fake_pose.cpp)
add_executable(${executable_door_opening_mechanism_simulation_motion_plan_request} src/door_opening_mechanism_simulation_motion_plan_request.cpp)

set(library_door_opening_mechanism_simulation ${executable_door_opening_mechanism_simulation}_core)

add_library(${library_door_opening_mechanism_simulation} SHARED
  src/door_opening_mechanism_simulation.cpp
)

set(dependencies
  rclcpp
  communication_interfaces
  depthai_ros_msgs
  moveit_ros_planning_interface
  graph_msgs
  rviz_visual_tools
  moveit_visual_tools
)

ament_target_dependencies(${executable_door_opening_mechanism_simulation_send_fake_pose}
  ${dependencies}
)
ament_target_dependencies(${executable_door_opening_mechanism_simulation_motion_plan_request}
  ${dependencies}
)
ament_target_dependencies(${executable_door_opening_mechanism_simulation}
  ${dependencies}
)
target_link_libraries(${executable_door_opening_mechanism_simulation} ${library_door_opening_mechanism_simulation})

ament_target_dependencies(${library_door_opening_mechanism_simulation}
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
  ${executable_door_opening_mechanism_simulation}
  ${executable_door_opening_mechanism_simulation_send_fake_pose}
  ${executable_door_opening_mechanism_simulation_motion_plan_request}
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
