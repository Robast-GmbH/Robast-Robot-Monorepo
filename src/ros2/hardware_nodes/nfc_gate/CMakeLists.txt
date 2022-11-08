cmake_minimum_required(VERSION 3.8)
project(nfc_gate)

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
find_package(rclcpp_action REQUIRED)
find_package(communication_interfaces REQUIRED)
find_package(ament_cmake_catch2 REQUIRED)
find_package(ament_cmake_uncrustify REQUIRED)
find_package(serial_helper REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED) 

include_directories(include)
include_directories(test/include)


set(executable_nfc_gate nfc_gate)
set(executable_nfc_gate_test nfc_gate_test)


add_executable(${executable_nfc_gate}
  src/main.cpp)

add_executable(${executable_nfc_gate_test}
  test/ros2_tests/main.cpp)

set(library_nfc_gate ${executable_nfc_gate}_core)

set(library_nfc_gate_test ${executable_nfc_gate_test}_core)

add_library(library_nfc_gate_test SHARED
  src/nfc_gate.cpp
  test/src/mock_serial_helper.cpp
  test/src/test_nfc_gate.cpp
)


add_library(library_nfc_gate SHARED
  src/nfc_gate.cpp
)

set(dependencies
  rclcpp
  rclcpp_action
  communication_interfaces
  serial_helper
)

ament_target_dependencies(${executable_nfc_gate}
  ${dependencies}
)

target_link_libraries(${executable_nfc_gate} library_nfc_gate)


ament_target_dependencies(library_nfc_gate
  ${dependencies}
)
ament_target_dependencies(${executable_nfc_gate_test}
  ${dependencies}
)

target_link_libraries(${executable_nfc_gate_test} library_nfc_gate_test)


ament_target_dependencies(library_nfc_gate_test
  ${dependencies}
)
target_include_directories(nfc_gate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(BUILD_TESTING)
  file(GLOB_RECURSE unit_test_srcs "test/unit_tests/*.cpp")

  file(GLOB_RECURSE test_srcs "test/src/*.cpp")

  add_launch_test(test/ros2_tests/node_test_launch_testing.py)

  ament_add_catch2(
    nfc_gate_unittest test/unit_tests/main.cpp ${test_srcs} ${unit_test_srcs}
  )

  ament_target_dependencies(nfc_gate_unittest
    ${dependencies}
  )

  target_link_libraries(nfc_gate_unittest
    library_nfc_gate
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
  )
  
  target_include_directories(nfc_gate_unittest
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/>
  )
endif()

install(FILES
  test/ros2_tests/node_test_launch_testing.py
  test/ros2_tests/node_test_expected_data.yaml 
  test/ros2_tests/node_test_input_data.yaml 
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  library_nfc_gate
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${executable_nfc_gate}
  ${executable_nfc_gate_test}
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
