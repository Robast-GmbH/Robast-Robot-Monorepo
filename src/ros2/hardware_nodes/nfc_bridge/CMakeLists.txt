cmake_minimum_required(VERSION 3.8)
project(nfc_bridge)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(communication_interfaces REQUIRED)
find_package(serial_helper REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  communication_interfaces
  serial_helper  
  std_srvs
)

set(executable_nfc_bridge nfc_bridge)

add_executable(${executable_nfc_bridge}
  src/main.cpp)

set(library_nfc_bridge ${executable_nfc_bridge}_core)

add_library(${library_nfc_bridge} SHARED
src/nfc_bridge.cpp
)

ament_target_dependencies(${executable_nfc_bridge}
  ${dependencies}
)
target_link_libraries(${executable_nfc_bridge} ${library_nfc_bridge})

ament_target_dependencies(${library_nfc_bridge}
  ${dependencies}
)

target_include_directories(nfc_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(BUILD_TESTING)

  include(FetchContent)

  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.6.0
  )
  FetchContent_MakeAvailable(Catch2)

  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8()
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
  find_package(ament_cmake_pep257 REQUIRED)
  ament_pep257()
  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck()

  add_executable(twn4Test test/twn4_test.cpp)
  target_link_libraries(twn4Test PRIVATE Catch2::Catch2WithMain) # Link against Catch2 and your library

  # Register tests
  include(CTest) # Enables testing
  include(Catch)
  catch_discover_tests(twn4Test)


endif()

install(TARGETS
${library_nfc_bridge}
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)

install(TARGETS
${executable_nfc_bridge}
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