cmake_minimum_required(VERSION VERSION 3.5)
project(gazebo-plugin-autonomous-actor2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package( ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)


link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(AutonomousActorPlugin2 SHARED src/AutonomousActorPlugin2.cc)
target_link_libraries(AutonomousActorPlugin2 ${caktin_LIBRARIES} ${GAZEBO_LIBRARIES})
ament_target_dependencies(AutonomousActorPlugin2 rclcpp)

# Debug Version (Note to add library before target compile definition)
add_library(AutonomousActorPlugin2Debug SHARED src/AutonomousActorPlugin2.cc)
target_compile_definitions(AutonomousActorPlugin2Debug PRIVATE DEBUG_)
target_link_libraries(AutonomousActorPlugin2Debug ${caktin_LIBRARIES} ${GAZEBO_LIBRARIES})
ament_package()