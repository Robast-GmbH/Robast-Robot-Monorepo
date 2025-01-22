cmake_minimum_required(VERSION 3.5)
project(error_utils)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Python REQUIRED COMPONENTS Interpreter Development)
find_package(communication_interfaces REQUIRED)

add_custom_target(run_script ALL
    COMMAND ${Python_EXECUTABLE} ${CMAKE_SOURCE_DIR}/generate_error_mapping.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)


# Add include directories
include_directories(include)

# Create library for C++ usage
add_library(${PROJECT_NAME} INTERFACE)

# Add dependencies to the library
ament_target_dependencies(${PROJECT_NAME} INTERFACE rclcpp)

if(BUILD_TESTING)
  include(FetchContent)

  FetchContent_Declare(
      Catch2
      GIT_REPOSITORY https://github.com/catchorg/Catch2.git
      GIT_TAG        v3.8.0
  )
  FetchContent_MakeAvailable(Catch2)
  
  add_executable(
      generic_error_converter_test test/test_generic_error_converter.cpp
  )

  target_link_libraries(generic_error_converter_test
      Catch2::Catch2
      Catch2::Catch2WithMain
  )

  set(test_dependencies
    communication_interfaces
  )

  ament_target_dependencies(generic_error_converter_test ${test_dependencies})
  include(Catch)
  catch_discover_tests(generic_error_converter_test)

  target_include_directories(generic_error_converter_test
      PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/>
  )

  include(CTest)

  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8()
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
  find_package(ament_cmake_pep257 REQUIRED)
  ament_pep257()
  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck()
endif()

# Install library (C++ headers)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_package()