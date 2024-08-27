cmake_minimum_required(VERSION 3.8)
project(dryve_d1_bridge)

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

set(executable_dryve_d1_bridge dryve_d1_bridge)
set(executable_shutdown_dryve_d1 shutdown_dryve_d1)

add_executable(${executable_dryve_d1_bridge}
  src/main.cpp
)
add_executable(${executable_shutdown_dryve_d1}
  src/shutdown_dryve_d1.cpp
)

add_library(library_dryve_d1_bridge SHARED
  src/dryve_d1_bridge.cpp
)

add_library(library_d1 SHARED
  src/d1.cpp
)

set(dependencies
  rclcpp
)

ament_target_dependencies(${executable_dryve_d1_bridge}
  ${dependencies}
)
target_link_libraries(${executable_dryve_d1_bridge} 
  library_dryve_d1_bridge
  library_d1
)
ament_target_dependencies(${executable_shutdown_dryve_d1}
  ${dependencies}
)
target_link_libraries(${executable_shutdown_dryve_d1} 
  library_d1
)

ament_target_dependencies(library_dryve_d1_bridge
  ${dependencies}
)

target_include_directories(${executable_dryve_d1_bridge} 
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_include_directories(${executable_shutdown_dryve_d1} 
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)


if(BUILD_TESTING)
  include(FetchContent)

  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.6.0
  )
  FetchContent_Declare(
    googletest 
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        b796f7d44681514f58a683a3a71ff17c94edb0c1 # should equal tag v1.13.0
  )
  FetchContent_MakeAvailable(Catch2 googletest)
  
  include_directories(test/include)

  list(APPEND test_libs library_d1)

  add_executable(test_d1 test/unit_tests/test_d1.cpp)
  list(APPEND test_executables test_d1)
  add_executable(test_si_unit_factor test/unit_tests/test_si_unit_factor.cpp)
  list(APPEND test_executables test_si_unit_factor)

  foreach(test ${test_executables})
    target_link_libraries(${test}
      ${library_name}
      ${plugin_libs}
      ${test_libs}
      Catch2::Catch2
      Catch2::Catch2WithMain
      gmock_main
    )
    ament_target_dependencies(${test}
      ${dependencies}
    )
    include(Catch)
    catch_discover_tests(${test})
  endforeach()

  include(CTest)

  # find_package(ament_cmake_lint_cmake REQUIRED)
  # ament_lint_cmake()
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
  library_dryve_d1_bridge
  library_d1
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${executable_dryve_d1_bridge}
  ${executable_shutdown_dryve_d1} 
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

ament_export_libraries(library_d1)

ament_package()