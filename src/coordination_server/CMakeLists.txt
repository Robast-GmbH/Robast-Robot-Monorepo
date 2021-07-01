cmake_minimum_required(VERSION 3.5)
project(coordination_server)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(rclcpp REQUIRED)



#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

#add_library(ActorPlugin SHARED plugins/ActorPlugin.cc)
#target_link_libraries(ActorPlugin ${GAZEBO_LIBRARIES})

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

#nav2_package()
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

install(DIRECTORY
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS  
 
  DESTINATION lib 
)



ament_package()
