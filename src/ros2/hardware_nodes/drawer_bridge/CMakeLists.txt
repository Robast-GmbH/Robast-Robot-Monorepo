cmake_minimum_required(VERSION 3.5)
project(drawer_bridge)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_uncrustify REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(communication_interfaces REQUIRED)
find_package(can REQUIRED)
find_package(module_config REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED) 
find_package(can_msgs REQUIRED)
find_package(error_utils REQUIRED)

include_directories(include)
include_directories(test/include)

set(executable_drawer_bridge drawer_bridge)
set(executable_drawer_bridge_test drawer_bridge_test)

add_executable(${executable_drawer_bridge}
  src/main.cpp)
add_executable(${executable_drawer_bridge_test}
  test/ros2_tests/main.cpp)

set(library_drawer_bridge ${executable_drawer_bridge}_core)
set(library_drawer_bridge_test ${executable_drawer_bridge_test}_core)

add_library(${library_drawer_bridge} SHARED
  src/drawer_bridge.cpp
  src/can_encoder_decoder.cpp
  src/can_message_creator.cpp
)
add_library(${library_drawer_bridge_test} SHARED
  src/drawer_bridge.cpp
  src/can_encoder_decoder.cpp
  src/can_message_creator.cpp
  test/src/test_drawer_bridge.cpp
)

set(dependencies
  rclcpp
  rclcpp_action
  communication_interfaces
  can
  can_msgs
  module_config
  error_utils
)

ament_target_dependencies(${executable_drawer_bridge}
  ${dependencies}
)
target_link_libraries(${executable_drawer_bridge} ${library_drawer_bridge})

ament_target_dependencies(${executable_drawer_bridge_test}
  ${dependencies}
)
target_link_libraries(${executable_drawer_bridge_test} ${library_drawer_bridge_test})

ament_target_dependencies(${library_drawer_bridge}
  ${dependencies}
)
ament_target_dependencies(${library_drawer_bridge_test}
  ${dependencies}
)

target_include_directories(drawer_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(BUILD_TESTING)
  file(GLOB_RECURSE test_srcs "test/src/*.cpp")

  add_launch_test(test/ros2_tests/node_test_launch_testing.py)
endif()

install(TARGETS
  ${library_drawer_bridge}
  ${library_drawer_bridge_test}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${executable_drawer_bridge}
  ${executable_drawer_bridge_test}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(FILES
  test/ros2_tests/node_test_launch_testing.py
  test/ros2_tests/node_test_expected_data.yaml 
  test/ros2_tests/node_test_input_data.yaml 
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)

ament_package()
