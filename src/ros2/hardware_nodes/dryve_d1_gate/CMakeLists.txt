cmake_minimum_required(VERSION 3.8)
project(dryve_d1_gate)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

set(executable_dryve_d1_gate dryve_d1_gate)

add_executable(${executable_dryve_d1_gate}
  src/main.cpp)

add_library(library_dryve_d1_gate SHARED
  src/dryve_d1_gate.cpp
)

add_library(library_d1 SHARED
  src/d1.cpp
)

set(dependencies
  rclcpp
)

ament_target_dependencies(${executable_dryve_d1_gate}
  ${dependencies}
)
target_link_libraries(${executable_dryve_d1_gate} 
  library_dryve_d1_gate
  library_d1
  )

ament_target_dependencies(library_dryve_d1_gate
  ${dependencies}
)

target_include_directories(dryve_d1_gate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  library_dryve_d1_gate
  library_d1
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${executable_dryve_d1_gate}
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