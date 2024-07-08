cmake_minimum_required(VERSION 3.16)
project(mobile_base_controller LANGUAGES CXX)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  generate_parameter_library
  controller_interface
  realtime_tools
  control_msgs
  std_msgs
  geometry_msgs
  nav_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(mobile_base_controller_parameters
  src/mobile_base_controller_parameters.yaml
)

add_library(mobile_base_controller SHARED
  src/mobile_base_controller.cpp
)
target_compile_features(mobile_base_controller PUBLIC cxx_std_17)
target_include_directories(mobile_base_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/mobile_base_controller>
)
target_link_libraries(mobile_base_controller PUBLIC
  mobile_base_controller_parameters
)
ament_target_dependencies(mobile_base_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(mobile_base_controller PRIVATE "MOBILE_BASE_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(controller_interface mobile_base_controller.xml)

install(
  DIRECTORY include/
  DESTINATION include/mobile_base_controller
)

install(TARGETS mobile_base_controller mobile_base_controller_parameters
  EXPORT export_mobile_base_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if("$ENV{ROS_DISTRO}" STREQUAL "humble")
  add_definitions(-DMOBILE_BASE_CONTROLLER__HUMBLE)
else()
  add_definitions(-DMOBILE_BASE_CONTROLLER__IRON)
endif()

ament_export_targets(export_mobile_base_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
