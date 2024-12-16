cmake_minimum_required(VERSION 3.5)
project(error_utils)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Python REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

# Add include directories
include_directories(include)

# Create library for C++ usage
add_library(${PROJECT_NAME} INTERFACE)

# Add dependencies to the library
ament_target_dependencies(${PROJECT_NAME} INTERFACE rclcpp)

# Install library (C++ headers)
install(DIRECTORY include/
  DESTINATION include
)
# Optionally, export Python bindings (if needed for other ROS packages)
ament_export_libraries(error_definitions_pybind)
# Export package information
ament_export_include_directories(include)
ament_package()

# Add Pybind11 module
add_library(error_definitions_pybind MODULE src/error_definitions_pybind.cpp)
target_include_directories(error_definitions_pybind PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(error_definitions_pybind PRIVATE pybind11::module)

# Ensure the Pybind11 module has the correct name and extension
set_target_properties(error_definitions_pybind PROPERTIES PREFIX "" SUFFIX ".so")

# Install Pybind11 module
install(TARGETS error_definitions_pybind
  LIBRARY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}
)


