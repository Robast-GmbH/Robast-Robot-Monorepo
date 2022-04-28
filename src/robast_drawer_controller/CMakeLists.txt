cmake_minimum_required(VERSION 3.5)
project(robast_drawer_controller)

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
find_package(robast_msgs REQUIRED)

include_directories(include)

set(executable_drawer_controller drawer_controller)

add_executable(${executable_drawer_controller}
  src/main.cpp)

set(library_drawer_controller ${executable_drawer_controller}_core)

add_library(${library_drawer_controller} SHARED
  src/drawer_controller.cpp
)

set(dependencies
  rclcpp
  rclcpp_action
  robast_msgs
)

ament_target_dependencies(${executable_drawer_controller}
  ${dependencies}
)

target_link_libraries(${executable_drawer_controller} ${library_drawer_controller})

ament_target_dependencies(${library_drawer_controller}
  ${dependencies}
)

target_include_directories(drawer_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  ${library_drawer_controller}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${executable_drawer_controller}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
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
