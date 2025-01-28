cmake_minimum_required(VERSION 3.5)
project(initial_pose_publisher)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

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

# Add and link tests
if(BUILD_TESTING)
  ament_add_gtest(test_initial_pose_publisher test/test_initial_pose_publisher.cpp)
  if(TARGET test_initial_pose_publisher)
    ament_target_dependencies(test_initial_pose_publisher rclcpp geometry_msgs)
  endif()
endif()

# Export dependencies
ament_package()
