cmake_minimum_required(VERSION 3.5)
project(hardware_interface_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(library_hardware_interface_utils SHARED
    ${CMAKE_CURRENT_LIST_DIR}/src/hardware_interface_utils.cpp
)

add_library(library_prismatic_joint_state_monitor SHARED
    ${CMAKE_CURRENT_LIST_DIR}/src/prismatic_joint_state_monitor.cpp
)

ament_target_dependencies(library_hardware_interface_utils
    rclcpp_lifecycle
    hardware_interface
    rclcpp
)

ament_target_dependencies(library_prismatic_joint_state_monitor
    rclcpp_lifecycle
    hardware_interface
    rclcpp
    nav_msgs
    geometry_msgs
    std_msgs
)

target_include_directories(library_hardware_interface_utils
    PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
    $<INSTALL_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_include_directories(library_prismatic_joint_state_monitor
    PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
    $<INSTALL_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

if(BUILD_TESTING)   
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
library_hardware_interface_utils
library_prismatic_joint_state_monitor
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include/  
)

ament_export_include_directories(include)

ament_export_libraries(library_hardware_interface_utils)
ament_export_libraries(library_prismatic_joint_state_monitor)

ament_package()