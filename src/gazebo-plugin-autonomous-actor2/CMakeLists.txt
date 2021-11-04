cmake_minimum_required(VERSION VERSION 3.5)
project(gazebo-plugin-autonomous-actor2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)

add_library(AutonomousActorPlugin2 SHARED src/AutonomousActorPlugin2.cc)

target_include_directories(AutonomousActorPlugin2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(AutonomousActorPlugin2
  "gazebo_ros"
  "rclcpp")

install(TARGETS
  AutonomousActorPlugin2
  DESTINATION share/../../../gaz/plugins)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()