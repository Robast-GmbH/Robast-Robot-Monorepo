cmake_minimum_required(VERSION 3.16)
project(mobile_base_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  control_msgs
  generate_parameter_library
  controller_interface
  parameter_traits
  realtime_tools
  std_msgs
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

# if(BUILD_TESTING)
#   find_package(ament_cmake_gmock REQUIRED)
#   find_package(controller_manager REQUIRED)
#   find_package(ros2_control_test_assets REQUIRED)

#   # Dynamically loaded during test
#   find_package(kinematics_interface_kdl REQUIRED)

#   # test loading admittance controller
#   add_rostest_with_parameters_gmock(test_load_mobile_base_controller
#     test/test_load_mobile_base_controller.cpp
#     ${CMAKE_CURRENT_SOURCE_DIR}/test/test_params.yaml
#   )
#   ament_target_dependencies(test_load_mobile_base_controller
#     controller_manager
#     hardware_interface
#     ros2_control_test_assets
#   )

#   # test admittance controller function
#   add_rostest_with_parameters_gmock(test_mobile_base_controller
#     test/test_mobile_base_controller.cpp
#     ${CMAKE_CURRENT_SOURCE_DIR}/test/test_params.yaml
#   )
#   target_link_libraries(test_mobile_base_controller mobile_base_controller)
#   ament_target_dependencies(test_mobile_base_controller
#     control_msgs
#     controller_interface
#     hardware_interface
#     ros2_control_test_assets
#   )
# endif()

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

ament_export_targets(export_mobile_base_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
