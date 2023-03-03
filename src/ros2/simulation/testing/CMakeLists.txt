cmake_minimum_required(VERSION 3.22)
project(testing)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(graph_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(moveit_visual_tools REQUIRED)

set(dependencies
  rclcpp
  moveit_ros_planning_interface
  graph_msgs
  rviz_visual_tools
  moveit_visual_tools
)

add_executable(move_main src/move_main.cpp)
target_include_directories(move_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move_main PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  move_main
  ${dependencies}
)
install(TARGETS move_main
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
ament_package()
