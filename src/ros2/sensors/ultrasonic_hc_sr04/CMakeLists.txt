cmake_minimum_required(VERSION 3.5)
project(ultrasonic_hc_sr04)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

set(executable_ultrasonic_hc_sr04 ultrasonic_hc_sr04)

add_executable(${executable_ultrasonic_hc_sr04}
  src/main.cpp)

set(library_ultrasonic_hc_sr04 ${executable_ultrasonic_hc_sr04}_core)

add_library(${library_ultrasonic_hc_sr04} SHARED
  src/ultrasonic_hc_sr04.cpp
)

set(dependencies
  rclcpp
  sensor_msgs
)

ament_target_dependencies(${executable_ultrasonic_hc_sr04}
  ${dependencies}
)
target_link_libraries(${executable_ultrasonic_hc_sr04} ${library_ultrasonic_hc_sr04} pigpiod_if2)

ament_target_dependencies(${library_ultrasonic_hc_sr04}
  ${dependencies}
)

target_include_directories(ultrasonic_hc_sr04 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  ${library_ultrasonic_hc_sr04}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${executable_ultrasonic_hc_sr04}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)

ament_package()
