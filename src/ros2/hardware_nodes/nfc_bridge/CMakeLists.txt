cmake_minimum_required(VERSION 3.8)
project(nfc_bridge)

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


set(executable_nfc_bridge nfc_bridge)
# set(executable_nfc_bridge_test nfc_bridge_test)

add_executable(${executable_nfc_bridge}
  src/main.cpp)
# add_executable(${executable_nfc_bridge_test}
#   test/ros2_tests/main.cpp)

set(library_nfc_bridge ${executable_nfc_bridge}_core)
# set(library_nfc_bridge_test ${executable_nfc_bridge_test}_core)

add_library(${library_nfc_bridge} SHARED
src/nfc_bridge.cpp
)
# add_library(${library_nfc_bridge_test} SHARED
#   src/nfc_bridge.cpp
#   test/src/mock_serial_helper.cpp
#   test/src/mock_postgresql_helper.cpp
#   test/src/test_nfc_bridge.cpp
# )
#ToDo: fix this that we can swap to the newer version
# include(FetchContent)
#   FetchContent_Declare(
#     libpqxx
#     GIT_REPOSITORY https://github.com/jtv/libpqxx.git
#     GIT_TAG        17e5a6c8ac3abc05329891aaa378bd6004b9c8ee # should equal tag v7.7.4
#   )
#   FetchContent_MakeAvailable(libpqxx)
#   file(REMOVE_RECURSE  CMAKE_BINARY_DIR/libpqxx-src/test/*)

set(dependencies
  rclcpp
  rclcpp_action
  communication_interfaces
  serial_helper
  db_helper
)

ament_target_dependencies(${executable_nfc_bridge}
  ${dependencies}
)
target_link_libraries(${executable_nfc_bridge} ${library_nfc_bridge})

# ament_target_dependencies(${executable_nfc_bridge_test}
#   ${dependencies}
# )
# target_link_libraries(${executable_nfc_bridge_test} ${library_nfc_bridge_test})

ament_target_dependencies(${library_nfc_bridge}
  ${dependencies}
)
# ament_target_dependencies(${library_nfc_bridge_test}
#   ${dependencies}
# )

target_include_directories(nfc_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(BUILD_TESTING)

  include(FetchContent)

  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v2.13.10 
  )
  FetchContent_MakeAvailable(Catch2)

  add_launch_test(test/ros2_tests/node_test_launch_testing.py)

  # add_executable(
  #   nfc_bridge_unit_test test/unit_tests/main.cpp test/unit_tests/test_cases.cpp ${test_srcs} ${unit_test_srcs}
  # )

  # ament_target_dependencies(nfc_bridge_unit_test
  #   ${dependencies}
  # )

  # target_link_libraries(nfc_bridge_unit_test
  #   ${library_nfc_bridge}
  #   ${library_nfc_bridge_test}
  #   rclcpp::rclcpp
  #   rclcpp_action::rclcpp_action
  #   Catch2::Catch2
  # )
  
  # target_include_directories(nfc_bridge_unit_test
  #   PRIVATE
  #   $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/>
  # )
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

# install(FILES
#   test/ros2_tests/node_test_launch_testing.py
#   test/ros2_tests/node_test_expected_data.yaml 
#   test/ros2_tests/node_test_input_data.yaml 
#   DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)

ament_package()