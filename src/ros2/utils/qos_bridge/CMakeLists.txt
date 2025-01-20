cmake_minimum_required(VERSION 3.8)
project(qos_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(communication_interfaces REQUIRED)
include_directories(include)
add_executable(error_qos_bridge src/error_qos_bridge.cpp)
ament_target_dependencies(error_qos_bridge rclcpp communication_interfaces)

install(TARGETS
  error_qos_bridge
  DESTINATION lib/${PROJECT_NAME})

ament_package()
