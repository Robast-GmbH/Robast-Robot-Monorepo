cmake_minimum_required(VERSION 3.8)
project(robot_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(fleet_interfaces REQUIRED)
find_package(communication_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

set(executable_robot_client robot_client)
add_executable(${executable_robot_client}
  src/main.cpp)

set(library_robot_client ${executable_robot_client}_core)
add_library(${library_robot_client} SHARED 
  src/robot_client.cpp
  src/action.cpp
  src/drawer_action.cpp 
  src/navigation_action.cpp
  src/nfc_action.cpp )

set(dependencies
  rclcpp
  rclcpp_action
  geometry_msgs
  tf2_geometry_msgs
  nav2_msgs
  fleet_interfaces
  communication_interfaces
  std_msgs
)

ament_target_dependencies(${executable_robot_client} ${dependencies})
target_link_libraries(${executable_robot_client} ${library_robot_client})

ament_target_dependencies(${library_robot_client}
  ${dependencies}
)


target_include_directories(robot_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


install(TARGETS
  ${executable_robot_client}
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()
