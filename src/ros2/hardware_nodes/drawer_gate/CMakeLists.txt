cmake_minimum_required(VERSION 3.5)
project(drawer_gate)

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
find_package(ament_cmake_catch2 REQUIRED)
find_package(ament_cmake_uncrustify REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(communication_interfaces REQUIRED)
find_package(can REQUIRED)
find_package(serial_helper REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED) 

include_directories(include)
include_directories(test/include)

set(executable_drawer_gate drawer_gate)
set(executable_drawer_gate_test drawer_gate_test)

add_executable(${executable_drawer_gate}
  src/main.cpp)
add_executable(${executable_drawer_gate_test}
  test/ros2_tests/main.cpp)

set(library_drawer_gate ${executable_drawer_gate}_core)
set(library_drawer_gate_test ${executable_drawer_gate_test}_core)

add_library(${library_drawer_gate} SHARED
  src/drawer_gate.cpp
)
add_library(${library_drawer_gate_test} SHARED
  src/drawer_gate.cpp
  test/src/mock_serial_helper.cpp
  test/src/test_drawer_gate.cpp
)

set(dependencies
  rclcpp
  rclcpp_action
  communication_interfaces
  can
  serial_helper
)

ament_target_dependencies(${executable_drawer_gate}
  ${dependencies}
)
target_link_libraries(${executable_drawer_gate} ${library_drawer_gate})

ament_target_dependencies(${executable_drawer_gate_test}
  ${dependencies}
)
target_link_libraries(${executable_drawer_gate_test} ${library_drawer_gate_test})

ament_target_dependencies(${library_drawer_gate}
  ${dependencies}
)
ament_target_dependencies(${library_drawer_gate_test}
  ${dependencies}
)

target_include_directories(drawer_gate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(BUILD_TESTING)
  file(GLOB_RECURSE unit_test_srcs "test/unit_tests/*.cpp")
  file(GLOB_RECURSE test_srcs "test/src/*.cpp")

  add_launch_test(test/ros2_tests/node_test_launch_testing.py)

  ament_add_catch2(
    drawer_gate_unit_test test/unit_tests/main.cpp ${test_srcs} ${unit_test_srcs}
  )

  ament_target_dependencies(drawer_gate_unit_test
    ${dependencies}
  )

  target_link_libraries(drawer_gate_unit_test
    ${library_drawer_gate}
    rclcpp::rclcpp
  )
  
  target_include_directories(drawer_gate_unit_test
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/>
  )
endif()

install(TARGETS
  ${library_drawer_gate}
  ${library_drawer_gate_test}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${executable_drawer_gate}
  ${executable_drawer_gate_test}
  RUNTIME DESTINATION lib/${PROJECT_NAME}1
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(FILES
  test/ros2_tests/node_test_launch_testing.py
  test/ros2_tests/node_test_expected_data.yaml 
  test/ros2_tests/node_test_input_data.yaml 
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)

ament_package()
