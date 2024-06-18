cmake_minimum_required(VERSION 3.22)
project(get_spatial_detections_from_topic CXX)

find_package(moveit_studio_common REQUIRED)
moveit_studio_package()

find_package(moveit_studio_behavior_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(depthai_ros_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
moveit_studio_behavior_interface
pluginlib
depthai_ros_msgs)

add_library(
  get_spatial_detections_from_topic
  SHARED
  src/get_spatial_detections_from_topic.cpp
  src/register_behaviors.cpp)
target_include_directories(
  get_spatial_detections_from_topic
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(get_spatial_detections_from_topic
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install Libraries
install(
  TARGETS get_spatial_detections_from_topic
  EXPORT get_spatial_detections_from_topicTargets
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
  moveit_studio_behavior_interface get_spatial_detections_from_topic_plugin_description.xml)

ament_export_targets(get_spatial_detections_from_topicTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
