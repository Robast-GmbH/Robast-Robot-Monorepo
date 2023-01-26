cmake_minimum_required(VERSION 3.8)
project(nfc_gate)

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


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(communication_interfaces REQUIRED)
find_package(ament_cmake_uncrustify REQUIRED)
find_package(serial_helper REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED) 
find_package(db_helper REQUIRED)

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

add_library(${library_nfc_gate} SHARED
src/nfc_gate.cpp
)
add_library(${library_nfc_gate_test} SHARED
  src/nfc_gate.cpp
  test/src/mock_serial_helper.cpp
  test/src/mock_postgresql_helper.cpp
  test/src/test_nfc_gate.cpp
)

set(dependencies
  rclcpp
  rclcpp_action
  communication_interfaces
  serial_helper
  db_helper
)

ament_target_dependencies(${executable_nfc_gate}
  ${dependencies}
)
target_link_libraries(${executable_nfc_gate} ${library_nfc_gate})

ament_target_dependencies(${executable_nfc_gate_test}
  ${dependencies}
)
target_link_libraries(${executable_nfc_gate_test} ${library_nfc_gate_test})

ament_target_dependencies(${library_nfc_gate}
  ${dependencies}
)
ament_target_dependencies(${library_nfc_gate_test}
  ${dependencies}
)

target_include_directories(nfc_gate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(BUILD_TESTING)

  include(FetchContent)

  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v2.13.10 # should equal tag v3.2.1
  )
  FetchContent_MakeAvailable(Catch2)

  file(GLOB_RECURSE unit_test_srcs "test/unit_tests/*.cpp")

  file(GLOB_RECURSE test_srcs "test/src/*.cpp")

  add_launch_test(test/ros2_tests/node_test_launch_testing.py)

  # ament_add_catch2(
  #   nfc_gate_unit_test test/unit_tests/main.cpp ${test_srcs} ${unit_test_srcs}
  # )
  add_executable(
    nfc_gate_unit_test test/unit_tests/main.cpp ${test_srcs} ${unit_test_srcs}
  )

  ament_target_dependencies(nfc_gate_unit_test
    ${dependencies}
  )

  target_link_libraries(nfc_gate_unit_test
    ${library_nfc_gate}
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    Catch2::Catch2
  )
  
  target_include_directories(nfc_gate_unit_test
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/>
  )
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8()
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
  find_package(ament_cmake_pep257 REQUIRED)
  ament_pep257()
  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck()
endif()

install(TARGETS
${library_nfc_gate}
${library_nfc_gate_test}
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

install(FILES
  test/ros2_tests/node_test_launch_testing.py
  test/ros2_tests/node_test_expected_data.yaml 
  test/ros2_tests/node_test_input_data.yaml 
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)

ament_package()