cmake_minimum_required(VERSION 3.22)
project(ros2_control_door_opening_mechanism_bringup)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})