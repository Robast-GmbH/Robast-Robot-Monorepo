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


include_directories(../serial_helper) # needed to find robast_msg/robast_can_msgs


include_directories(include)

set(executable_nfc_gate nfc_gate)

add_executable(${executable_nfc_gate}
  src/main.cpp)

set(library_nfc_gate ${executable_nfc_gate}_core)

add_library(${library_nfc_gate} SHARED
  src/nfc_gate.cpp
  ../serial_helper/src/serial_helper.cpp #TODO: This can probably be done better
)



set(dependencies
  rclcpp
  rclcpp_action
  communication_interfaces
)

ament_target_dependencies(${executable_nfc_gate}
  ${dependencies}
)

target_link_libraries(${executable_nfc_gate} ${library_nfc_gate})


ament_target_dependencies(${library_nfc_gate}
  ${dependencies}
)

target_include_directories(nfc_gate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  ${library_nfc_gate}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${executable_nfc_gate}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()