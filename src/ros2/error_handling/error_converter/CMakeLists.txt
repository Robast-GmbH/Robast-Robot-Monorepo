cmake_minimum_required(VERSION 3.10.2)
project(error_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(communication_interfaces REQUIRED)

set(dependencies
  rclcpp
  std_msgs
)

include_directories(
  include
)

ament_target_dependencies(drawer_sm_startup_debug
  ${dependencies}
)

target_include_directories(drawer_sm_startup_debug
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS
drawer_sm_startup_debug
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY

DESTINATION share/${PROJECT_NAME})


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
endif()


ament_export_include_directories(include)

ament_package()

