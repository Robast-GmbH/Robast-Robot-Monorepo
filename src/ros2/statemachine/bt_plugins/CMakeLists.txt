cmake_minimum_required(VERSION 3.8)
project(bt_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(communication_interfaces REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  communication_interfaces
  behaviortree_cpp_v3
)
include_directories(
  include
)
set(lib_name bt_plugin_core)


# add_library(drawer_open_req STATIC 
#   src/action/drawer_open_request_action.cpp)
# add_library(open_drawer STATIC 
#   src/action/open_drawer_action.cpp)
# add_library(change_leds STATIC 
#   src/action/change_led_action.cpp)
# add_library(drawer_status_condition STATIC 
#   src/condition/drawer_status_condition.cpp)

add_library(${lib_name} SHARED
  src/action/drawer_open_request_action.cpp
  src/action/open_drawer_action.cpp
  src/action/change_led_action.cpp
  src/condition/drawer_status_condition.cpp
)
# target_link_libraries(drawer_open_req PUBLIC behaviortree_cpp_v3)
# target_include_directories(${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# ament_target_dependencies(drawer_open_req ${dependencies})
# ament_target_dependencies(open_drawer ${dependencies})
# ament_target_dependencies(change_leds ${dependencies})
# ament_target_dependencies(drawer_status_condition ${dependencies})

ament_target_dependencies(${lib_name}
  ${dependencies}
)
# rclcpp_components_register_nodes(${lib_name} "nav2_bt_navigator::BtNavigator")

install(TARGETS ${lib_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_include_directories(include)
ament_export_libraries(${lib_name})
ament_export_dependencies(${dependencies})

ament_package()
