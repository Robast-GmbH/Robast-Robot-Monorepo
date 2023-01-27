cmake_minimum_required(VERSION 3.8)
project(door_manipulator_gate)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(communication_interfaces REQUIRED)
find_package(can REQUIRED)
find_package(serial_helper REQUIRED)

include_directories(include)

set(executable_door_manipulator_gate door_manipulator_gate)

add_executable(${executable_door_manipulator_gate}
  src/main.cpp)

set(library_door_manipulator_gate ${executable_door_manipulator_gate}_core)

add_library(${library_door_manipulator_gate} SHARED
  src/door_manipulator_gate.cpp
)

set(dependencies
  rclcpp
  communication_interfaces
  can
  serial_helper
)

ament_target_dependencies(${executable_door_manipulator_gate}
  ${dependencies}
)
target_link_libraries(${executable_door_manipulator_gate} ${library_door_manipulator_gate})

ament_target_dependencies(${library_door_manipulator_gate}
  ${dependencies}
)

target_include_directories(door_manipulator_gate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  ${library_door_manipulator_gate}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${executable_door_manipulator_gate}
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
