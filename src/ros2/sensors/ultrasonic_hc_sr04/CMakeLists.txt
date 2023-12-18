cmake_minimum_required(VERSION 3.5)
project(ultrasonic_hc_sr04)

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
find_package(ament_cmake_uncrustify REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

set(executable_drawer_bridge drawer_bridge)
set(executable_drawer_bridge_test drawer_bridge_test)

add_executable(${executable_drawer_bridge}
  src/main.cpp)

set(library_drawer_bridge ${executable_drawer_bridge}_core)

add_library(${library_drawer_bridge} SHARED
  src/ultrasonic_hc_sr04.cpp
)

set(dependencies
  rclcpp
  rclcpp_action
)

ament_target_dependencies(${executable_drawer_bridge}
  ${dependencies}
)
target_link_libraries(${executable_drawer_bridge} ${library_drawer_bridge})

ament_target_dependencies(${library_drawer_bridge}
  ${dependencies}
)

target_include_directories(drawer_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  ${library_drawer_bridge}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${executable_drawer_bridge}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)

ament_package()
