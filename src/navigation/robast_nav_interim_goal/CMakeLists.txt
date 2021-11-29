cmake_minimum_required(VERSION 3.5)
project(robast_nav_interim_goal)

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
find_package(rclcpp_lifecycle REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(robast_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

set(executable_goal_selector interim_goal_selector)
set(executable_poses_importer yaml_poses_importer)

add_executable(${executable_goal_selector}
  src/interim_goal_selector/main.cpp)
add_executable(${executable_poses_importer}
  src/yaml_poses_importer/main.cpp)

set(library_goal_selector ${executable_goal_selector}_core)
set(library_poses_importer ${executable_poses_importer}_core)

add_library(${library_goal_selector} SHARED
  src/interim_goal_selector/interim_goal_selector.cpp
)
add_library(${library_poses_importer} SHARED
  src/yaml_poses_importer/yaml_poses_importer.cpp
)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  yaml_cpp_vendor
  nav2_util
  robast_msgs
  tf2
  tf2_geometry_msgs
)

ament_target_dependencies(${executable_goal_selector}
  ${dependencies}
)
ament_target_dependencies(${executable_poses_importer}
  ${dependencies}
)

target_link_libraries(${executable_goal_selector} ${library_goal_selector})
target_link_libraries(${executable_poses_importer} ${library_poses_importer})

ament_target_dependencies(${library_goal_selector}
  ${dependencies}
)
ament_target_dependencies(${library_poses_importer}
  ${dependencies}
)

install(TARGETS
  ${library_goal_selector}
  ${library_poses_importer}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${executable_goal_selector}
  ${executable_poses_importer}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()
