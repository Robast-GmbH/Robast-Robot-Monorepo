cmake_minimum_required(VERSION VERSION 3.5)
project(gazebo-plugin-autonomous-actor)

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

add_library(AutonomousActorPlugin SHARED src/AutonomousActorPlugin.cc)
add_library(CollisionActorPlugin SHARED src/CollisionActorPlugin.cc)

target_include_directories(AutonomousActorPlugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(CollisionActorPlugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(AutonomousActorPlugin
  "gazebo_ros"
  "rclcpp")

  ament_target_dependencies(CollisionActorPlugin
  "gazebo_ros"
  "rclcpp")

install(TARGETS
  AutonomousActorPlugin
  DESTINATION share/../../../gaz/plugins)

  install(TARGETS
  CollisionActorPlugin
  DESTINATION share/../../../gaz/plugins)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()