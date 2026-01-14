cmake_minimum_required(VERSION 3.5)
project(behavior_tree_plugins)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
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
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(communication_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)


include_directories(
  include
)

set(library_name ${PROJECT_NAME})


set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  nav2_msgs
  behaviortree_cpp_v3
  std_msgs
  std_srvs
  nav2_util
  communication_interfaces
  geometry_msgs
  nav_msgs
)

# ament_target_dependencies(${library_name}
#   ${dependencies}
# )

# ACTION NODES
add_library(interim_goal_selector_action_bt_node SHARED plugins/action/interim_goal_selector_action.cpp)
list(APPEND plugin_libs interim_goal_selector_action_bt_node)
add_library(yaml_poses_import_action_bt_node SHARED plugins/action/yaml_poses_import_action.cpp)
list(APPEND plugin_libs yaml_poses_import_action_bt_node)
add_library(change_footprint_action_bt_node SHARED plugins/action/change_footprint_action.cpp)
list(APPEND plugin_libs change_footprint_action_bt_node)
add_library(change_footprint_padding_action_bt_node SHARED plugins/action/change_footprint_padding_action.cpp)
list(APPEND plugin_libs change_footprint_padding_action_bt_node)
add_library(play_sound_action_bt_node SHARED plugins/action/play_sound_action.cpp)
list(APPEND plugin_libs play_sound_action_bt_node)

# CONDITION NODES
add_library(check_state_condition_bt_node SHARED plugins/condition/check_state_condition.cpp)
list(APPEND plugin_libs check_state_condition_bt_node)
add_library(check_topic_state_condition_bt_node SHARED plugins/condition/check_topic_state_condition.cpp)
list(APPEND plugin_libs check_topic_state_condition_bt_node)
add_library(variable_update_condition_bt_node SHARED plugins/condition/variable_condition_updated.cpp)
list(APPEND plugin_libs variable_update_condition_bt_node)
add_library(is_incremented_condition_bt_node SHARED plugins/condition/is_incremented_condition.cpp)
list(APPEND plugin_libs is_incremented_condition_bt_node)
add_library(robast_goal_updated_condition_bt_node SHARED plugins/condition/robast_goal_updated_condition.cpp)
list(APPEND plugin_libs robast_goal_updated_condition_bt_node)
add_library(compare_integer_condition_bt_node SHARED plugins/condition/compare_integer_condition.cpp)
list(APPEND plugin_libs compare_integer_condition_bt_node)
add_library(string_topic_condition_bt_node SHARED plugins/condition/string_topic_condition.cpp)
list(APPEND plugin_libs string_topic_condition_bt_node)
add_library(check_person_in_front_condition_bt_node SHARED plugins/condition/check_person_in_front_condition.cpp)
list(APPEND plugin_libs check_person_in_front_condition_bt_node)

# DECORATER NODES
add_library(revert_path_node_decorator_bt_node SHARED plugins/decorator/revert_path_node.cpp)
list(APPEND plugin_libs revert_path_node_decorator_bt_node)


foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES xml/bt_nodes.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${library_name}
  ${plugin_libs}
)

ament_export_dependencies(${dependencies})

ament_package()
