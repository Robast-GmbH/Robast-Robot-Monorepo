cmake_minimum_required(VERSION 3.8)
project(clicked_point_to_nav_pose)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

include_directories(include)

set(executable_clicked_point_to_nav_pose clicked_point_to_nav_pose)

add_executable(${executable_clicked_point_to_nav_pose}
src/main.cpp)

set(library_clicked_point_to_nav_pose ${executable_clicked_point_to_nav_pose}_core)

add_library(${library_clicked_point_to_nav_pose} SHARED
  src/clicked_point_to_nav_pose.cpp
)

set(dependencies
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
)

ament_target_dependencies(${executable_clicked_point_to_nav_pose}
  ${dependencies}
)
target_link_libraries(${executable_clicked_point_to_nav_pose} ${library_clicked_point_to_nav_pose})

ament_target_dependencies(${library_clicked_point_to_nav_pose}
  ${dependencies}
)

target_include_directories(clicked_point_to_nav_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  ${library_clicked_point_to_nav_pose}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${executable_clicked_point_to_nav_pose}
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
