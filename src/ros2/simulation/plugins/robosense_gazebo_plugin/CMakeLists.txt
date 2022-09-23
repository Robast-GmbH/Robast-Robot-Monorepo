cmake_minimum_required(VERSION 3.5)
project(robosense_gazebo_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

add_library( GazeboRosRobosenseLaser SHARED src/GazeboRosRobosenseLaser.cpp)
ament_target_dependencies( GazeboRosRobosenseLaser
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "sensor_msgs"
)
ament_export_libraries( GazeboRosRobosenseLaser)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_ros)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
    GazeboRosRobosenseLaser
    DESTINATION share/../../../gaz/plugins
)
