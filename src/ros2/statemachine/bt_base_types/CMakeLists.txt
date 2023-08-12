cmake_minimum_required(VERSION 3.5)
project(bt_base_types)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(communication_interfaces REQUIRED)

# Add include directories
include_directories(include)

# Create library
add_library(${PROJECT_NAME} INTERFACE)

# Add dependencies to the library
ament_target_dependencies(${PROJECT_NAME} INTERFACE rclcpp communication_interfaces)

# Install library
install(DIRECTORY include/
  DESTINATION include
)

# Export package information
ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME})
ament_package()
