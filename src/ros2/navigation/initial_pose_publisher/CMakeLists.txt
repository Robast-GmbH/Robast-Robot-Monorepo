cmake_minimum_required(VERSION 3.5)
project(initial_pose_publisher)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add the executable
add_executable(initial_pose_publisher src/initial_pose_publisher.cpp)

# Link libraries
ament_target_dependencies(initial_pose_publisher rclcpp geometry_msgs)

# Install the executable
install(TARGETS
  initial_pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Export dependencies
ament_package()
