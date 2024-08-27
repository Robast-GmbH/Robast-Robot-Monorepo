cmake_minimum_required(VERSION 3.5)
project(gz_ros2_control_base_movement)

# Default to C11
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface_utils REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

if("$ENV{GZ_VERSION}" STREQUAL "garden" OR "$ENV{GZ_VERSION}" STREQUAL "harmonic")
  find_package(gz_ros2_control REQUIRED)
  # The `find_package(gz_ros2_control REQUIRED)` line above is supposed to find the `gz_ros2_control`
  # package and set several variables, including `gz_ros2_control_INCLUDE_DIRS`, which should contain the
  # path(s) to the package's include files.
  include_directories(include ${gz_ros2_control_INCLUDE_DIRS}/include)
else()
  find_package(ign_ros2_control REQUIRED)
  include_directories(include ${ign_ros2_control_INCLUDE_DIRS}/include)
endif()

set(GZ_PLUGIN)
set(GZ_SIM)

if("$ENV{GZ_VERSION}" STREQUAL "garden")
  find_package(gz-sim7 REQUIRED)
  set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo Garden")
  find_package(gz-plugin2 REQUIRED)
  set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
  set(GZ_PLUGIN gz-plugin${GZ_PLUGIN_VER}::register)
  set(GZ_SIM gz-sim${GZ_SIM_VER}::core)
  add_definitions(-DGZ_HEADERS)
elseif("$ENV{GZ_VERSION}" STREQUAL "harmonic")
  find_package(gz-sim8 REQUIRED)
  set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo Harmonic")
  find_package(gz-plugin2 REQUIRED)
  set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
  set(GZ_PLUGIN gz-plugin${GZ_PLUGIN_VER}::register)
  set(GZ_SIM gz-sim${GZ_SIM_VER}::core)
  add_definitions(-DGZ_HEADERS)
else()
  find_package(ignition-gazebo6 REQUIRED)
  set(GZ_SIM_VER ${ignition-gazebo6_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo Fortress")
  find_package(ignition-plugin1 REQUIRED)
  set(GZ_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
  set(GZ_PLUGIN ignition-plugin${GZ_PLUGIN_VER}::register)
  set(GZ_SIM ignition-gazebo${GZ_SIM_VER}::core)
endif()

include_directories(include)

## GARDEN or HARMONIC
if("$ENV{GZ_VERSION}" STREQUAL "garden" OR "$ENV{GZ_VERSION}" STREQUAL "harmonic")
  add_library(gz_ros2_control_base_movement SHARED
    src/gz_ros2_control_base_movement.cpp
  )

  ament_target_dependencies(gz_ros2_control_base_movement
    gz_ros2_control
    rclcpp_lifecycle
    hardware_interface
    rclcpp
    hardware_interface_utils
    nav_msgs
    geometry_msgs
  )

  target_link_libraries(gz_ros2_control_base_movement
    ${GZ_SIM}
  )

  install(TARGETS
    gz_ros2_control_base_movement
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

else() ## FORTRESS
  add_library(ign_ros2_control_base_movement SHARED
    src/ign_ros2_control_base_movement.cpp
  )

  ament_target_dependencies(ign_ros2_control_base_movement
    ign_ros2_control
    rclcpp_lifecycle
    hardware_interface
    rclcpp
    hardware_interface_utils
    nav_msgs
    geometry_msgs
  )

  target_link_libraries(ign_ros2_control_base_movement
    ${GZ_SIM}
  )

  install(TARGETS
    ign_ros2_control_base_movement
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endif()

install(DIRECTORY
  include
  DESTINATION include
)

# Testing and linting
if(BUILD_TESTING)
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

ament_export_include_directories(include)

## GARDEN or HARMONIC
if("$ENV{GZ_VERSION}" STREQUAL "garden" OR "$ENV{GZ_VERSION}" STREQUAL "harmonic")
  ament_export_libraries(${PROJECT_NAME} gz_ros2_control_base_movement)
  pluginlib_export_plugin_description_file(gz_ros2_control gz_ros2_control_base_movement.xml)
else() ## FORTRESS
  ament_export_libraries(${PROJECT_NAME} ign_ros2_control_base_movement)
  pluginlib_export_plugin_description_file(ign_ros2_control ign_ros2_control_base_movement.xml)
endif()

ament_package()
