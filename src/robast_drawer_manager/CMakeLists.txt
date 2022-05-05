cmake_minimum_required(VERSION 3.5)
project(robast_drawer_manager)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robast_ros2_msgs REQUIRED)

include_directories(include)

set(executable_drawer_manager drawer_manager)

add_executable(${executable_drawer_manager}
  src/main.cpp)

set(library_drawer_manager ${executable_drawer_manager}_core)

add_library(${library_drawer_manager} SHARED
  src/drawer_manager.cpp
)

set(dependencies
  rclcpp
  robast_hardware_nodes_msgs
  rclcpp_action
)

ament_target_dependencies(${executable_drawer_manager}
  ${dependencies}
)

target_link_libraries(${executable_drawer_manager} ${library_drawer_manager})

ament_target_dependencies(${library_drawer_manager}
  ${dependencies}
)

target_include_directories(drawer_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  ${library_drawer_manager}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${executable_drawer_manager}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()