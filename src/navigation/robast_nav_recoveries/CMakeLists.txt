cmake_minimum_required(VERSION 3.8)
project(robast_nav_recoveries)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

include_directories(include)

set(executable_recoveries_costmap recoveries_costmap)

add_executable(${executable_recoveries_costmap}
  src/recoveries_costmap/main.cpp)

set(library_recoveries_costmap ${executable_recoveries_costmap}_core)

add_library(${library_recoveries_costmap} SHARED
  src/recoveries_costmap/recoveries_costmap.cpp
)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  nav2_util
  nav2_costmap_2d
  rclcpp_components
)

ament_target_dependencies(${executable_recoveries_costmap}
  ${dependencies}
)

target_link_libraries(${executable_recoveries_costmap} ${library_recoveries_costmap})

ament_target_dependencies(${library_recoveries_costmap}
  ${dependencies}
)

# rclcpp_components_register_nodes(${library_name} "robast_nav_recoveries::RecoveriesCostmap")

install(TARGETS
  ${executable_recoveries_costmap}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
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
