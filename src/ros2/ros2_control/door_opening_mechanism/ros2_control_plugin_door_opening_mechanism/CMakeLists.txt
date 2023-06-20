cmake_minimum_required(VERSION 3.16)
project(ros2_control_plugin_door_opening_mechanism)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  dryve_d1_bridge
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  ros2_control_plugin_door_opening_mechanism
  SHARED
  src/door_opening_mechansim.cpp
)
target_compile_features(ros2_control_plugin_door_opening_mechanism PUBLIC cxx_std_17)
target_include_directories(ros2_control_plugin_door_opening_mechanism PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/ros2_control_plugin_door_opening_mechanism>
)
ament_target_dependencies(
  ros2_control_plugin_door_opening_mechanism PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROS_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ros2_control_plugin_door_opening_mechanism.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/ros2_control_plugin_door_opening_mechanism
)
install(TARGETS ros2_control_plugin_door_opening_mechanism
  EXPORT export_ros2_control_plugin_door_opening_mechanism
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_ros2_control_plugin_door_opening_mechanism HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
