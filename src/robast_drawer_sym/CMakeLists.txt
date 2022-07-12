cmake_minimum_required(VERSION 3.8)
project(robast_drawer_sym)

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
find_package(std_msgs REQUIRED)


include_directories(include)

set(executable_drawer_sym drawer_sym )

add_executable(${executable_drawer_sym}
  src/main.cpp)

set(library_drawer_sym  ${executable_drawer_sym}_core)

add_library(${library_drawer_sym} SHARED
  src/drawer_sym.cpp
)

set(dependencies
  rclcpp
  rclcpp_action
  robast_ros2_msgs
)

ament_target_dependencies(${executable_drawer_sym}
  ${dependencies}
)

target_link_libraries(${executable_drawer_sym} ${library_drawer_sym})


ament_target_dependencies(${library_drawer_sym}
  ${dependencies}
)

target_include_directories(drawer_sym PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  ${library_drawer_sym}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${executable_drawer_sym}
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