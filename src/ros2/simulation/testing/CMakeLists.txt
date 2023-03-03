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

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
ament_package()
