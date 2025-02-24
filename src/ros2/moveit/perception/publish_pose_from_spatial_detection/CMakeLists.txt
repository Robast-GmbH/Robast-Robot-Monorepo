cmake_minimum_required(VERSION 3.8)
project(publish_pose_from_spatial_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies_list
  ament_cmake
  rclcpp
  depthai_ros_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

foreach(dependency ${dependencies_list})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(
  include
)

set(executable_publish_pose_from_spatial_detection publish_pose_from_spatial_detection)
set(library_publish_pose_from_spatial_detection ${executable_publish_pose_from_spatial_detection}_core)

add_executable(${executable_publish_pose_from_spatial_detection}
  src/publish_pose_from_spatial_detection_main.cpp)

add_library(${library_publish_pose_from_spatial_detection} SHARED
  src/publish_pose_from_spatial_detection.cpp
)
  
ament_target_dependencies(${executable_publish_pose_from_spatial_detection}
  ${dependencies_list}
)
target_link_libraries(${executable_publish_pose_from_spatial_detection} ${library_publish_pose_from_spatial_detection})

ament_target_dependencies(${library_publish_pose_from_spatial_detection}
  ${dependencies_list}
)

target_include_directories(${executable_publish_pose_from_spatial_detection} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  ${executable_publish_pose_from_spatial_detection}
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${library_publish_pose_from_spatial_detection}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(${library_publish_pose_from_spatial_detection})

ament_export_include_directories(include)

ament_package()
