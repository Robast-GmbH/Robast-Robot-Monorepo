cmake_minimum_required(VERSION 3.8)
project(gazebo_controller_manager)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(gz_transport_version gz-transport12)
set(gz_msgs_version gz-msgs9)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(${gz_transport_version} REQUIRED)
find_package(${gz_msgs_version} REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  control_msgs
  ${gz_transport_version}
  ${gz_msgs_version}
)

include_directories(include)

set(executable_joint_position_controller joint_position_controller)
set(executable_joint_trajectory_controller joint_trajectory_controller)

add_executable(${executable_joint_position_controller}
  src/joint_position_controller_main.cpp)
add_executable(${executable_joint_trajectory_controller}
  src/joint_trajectory_controller_main.cpp)

set(library_joint_position_controller ${executable_joint_position_controller}_core)
set(library_joint_trajectory_controller ${executable_joint_trajectory_controller}_core)

add_library(${library_joint_position_controller} SHARED
  src/joint_position_controller.cpp
)
add_library(${library_joint_trajectory_controller} SHARED
  src/joint_trajectory_controller.cpp
)

ament_target_dependencies(${executable_joint_position_controller}
  ${dependencies}
)
target_link_libraries(${executable_joint_position_controller} ${library_joint_position_controller})

ament_target_dependencies(${executable_joint_trajectory_controller}
  ${dependencies}
)
target_link_libraries(${executable_joint_trajectory_controller} ${library_joint_trajectory_controller})


ament_target_dependencies(${library_joint_position_controller}
  ${dependencies}
)
ament_target_dependencies(${library_joint_trajectory_controller}
  ${dependencies}
)

target_include_directories(${executable_joint_position_controller} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(${executable_joint_trajectory_controller} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  ${executable_joint_position_controller}
  ${executable_joint_trajectory_controller}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)

ament_package()

