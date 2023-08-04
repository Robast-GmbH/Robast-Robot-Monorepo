cmake_minimum_required(VERSION 3.8)
project(behavior_tree_server_collection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(communication_interfaces REQUIRED)

include_directories(include)

set(executable_bt_server_collection bt_server_collection)

add_executable(${executable_bt_server_collection}
  src/main.cpp)

set(library_bt_server_collection ${executable_bt_server_collection}_core)

add_library(${library_bt_server_collection} SHARED
  src/bt_server_collection.cpp
)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  rclcpp_lifecycle
  nav2_util
  communication_interfaces
)

ament_target_dependencies(${executable_bt_server_collection}
  ${dependencies}
)

target_link_libraries(${executable_bt_server_collection} ${library_bt_server_collection})

ament_target_dependencies(${library_bt_server_collection}
  ${dependencies}
)

install(TARGETS
  ${library_bt_server_collection}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${executable_bt_server_collection}
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