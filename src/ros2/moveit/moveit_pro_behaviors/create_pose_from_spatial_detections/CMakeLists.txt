cmake_minimum_required(VERSION 3.22)
project(create_pose_from_spatial_detections CXX)

find_package(moveit_studio_common REQUIRED)
moveit_studio_package()

find_package(moveit_studio_behavior_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(depthai_ros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
moveit_studio_behavior_interface
pluginlib
tf2
tf2_ros
tf2_geometry_msgs
depthai_ros_msgs
geometry_msgs)

add_library(
  create_pose_from_spatial_detections
  SHARED
  src/create_pose_from_spatial_detections.cpp
  src/register_behaviors.cpp)
target_include_directories(
  create_pose_from_spatial_detections
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(create_pose_from_spatial_detections
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install Libraries
install(
  TARGETS create_pose_from_spatial_detections
  EXPORT create_pose_from_spatial_detectionsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  add_subdirectory(test)
  ament_lint_auto_find_test_dependencies()
endif()

# Export the behavior plugins defined in this package so they are available to
# plugin loaders that load the behavior base class library from the
# moveit_studio_behavior package.
pluginlib_export_plugin_description_file(
  moveit_studio_behavior_interface create_pose_from_spatial_detections_plugin_description.xml)

ament_export_targets(create_pose_from_spatial_detectionsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
