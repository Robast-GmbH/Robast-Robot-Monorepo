cmake_minimum_required(VERSION 3.5)
project(integration_tests_pkg)

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


if(BUILD_TESTING)
  ament_add_test(test_nfc_drawer
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/src/drawer_sys_test/test_case_launch.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    TIMEOUT 45
    ENV
      TEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}/src/drawer_sys_test
  )
  ament_add_test(test_waypoint_follower
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/src/nav_tests/test_case_launch.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    TIMEOUT 210
    ENV
      TEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}/src/nav_tests
  )
endif()

ament_package()

