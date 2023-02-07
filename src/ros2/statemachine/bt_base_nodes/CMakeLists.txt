cmake_minimum_required(VERSION 3.10.2)
project(bt_base_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(communication_interfaces REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(bt_plugins REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  communication_interfaces
  behaviortree_cpp
  bt_plugins
)

include_directories(
  include
)

add_executable(drawer_tree_initiator src/bt_sub_initiator/drawer_tree_initiator.cpp)
add_executable(drawer_nfc_tree_initiator src/bt_sub_initiator/drawer_nfc_tree_initiator.cpp)


ament_target_dependencies(drawer_tree_initiator
  ${dependencies}
)

target_include_directories(drawer_tree_initiator
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(drawer_nfc_tree_initiator
  ${dependencies}
)

target_include_directories(drawer_nfc_tree_initiator
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS
drawer_tree_initiator
drawer_nfc_tree_initiator
DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
include/
DESTINATION
include/)


ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8()
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
  find_package(ament_cmake_pep257 REQUIRED)
  ament_pep257()
  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck()
endif()

ament_package()
