cmake_minimum_required(VERSION 3.22)
project(moveit_robast_perception)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

option(WITH_OPENGL "Build the parts that depend on OpenGL" ON)

if(WITH_OPENGL)
  # Prefer newer vendor-specific OpenGL library
  if(POLICY CMP0072)
    cmake_policy(SET CMP0072 NEW)
  endif()
  find_package(OpenGL REQUIRED)
  find_package(GLEW REQUIRED)

  set(gl_LIBS ${gl_LIBS} ${OPENGL_LIBRARIES})
  if(APPLE)
    find_package(FreeGLUT REQUIRED)
    set(SYSTEM_GL_LIBRARIES ${GLEW_LIBRARIES} GLEW::GLEW FreeGLUT::freeglut)
  else()
    find_package(GLUT REQUIRED)
    if(WIN32)
      set(SYSTEM_GL_LIBRARIES GLEW::glew GLUT::GLUT)
    else()
      set(SYSTEM_GL_LIBRARIES ${GLEW_LIBRARIES} GLUT::GLUT)
    endif()
  endif()
  set(SYSTEM_GL_INCLUDE_DIRS ${GLEW_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})
endif()

if(APPLE)
  find_package(X11 REQUIRED)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_ros_occupancy_map_monitor REQUIRED)
find_package(moveit_ros_perception REQUIRED)

# Finds Boost Components
include(ConfigExtras.cmake)

set(THIS_PACKAGE_INCLUDE_DIRS
  robast_pointcloud_octomap_updater/include
  ${perception_GL_INCLUDE_DIRS}
)

set(THIS_PACKAGE_LIBRARIES
  moveit_robast_pointcloud_octomap_updater
  moveit_robast_pointcloud_octomap_updater_core
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_core
  moveit_ros_occupancy_map_monitor
  moveit_ros_perception
  rclcpp
  sensor_msgs
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${SYSTEM_GL_INCLUDE_DIR}
  ${X11_INCLUDE_DIR}
)

add_subdirectory(robast_pointcloud_octomap_updater)
add_subdirectory(publish_pose_from_spatial_detection)

install(
  TARGETS ${THIS_PACKAGE_LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(moveit_ros_occupancy_map_monitor "robast_pointcloud_octomap_updater_plugin_description.xml")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(CONFIG_EXTRAS ConfigExtras.cmake)
