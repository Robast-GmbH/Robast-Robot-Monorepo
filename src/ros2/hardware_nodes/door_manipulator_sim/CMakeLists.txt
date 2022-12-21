cmake_minimum_required(VERSION 3.8)
project(door_manipulator_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(communication_interfaces REQUIRED)


include_directories(include)


set(executable_door_manipulator_sim door_manipulator_sim)

add_executable(${executable_door_manipulator_sim}
  src/main.cpp)

  set(library_door_manipulator_sim ${executable_door_manipulator_sim}_core)

  add_library(${library_door_manipulator_sim} SHARED
    src/door_manipulator_sim.cpp
  )

set(dependencies
  rclcpp
  communication_interfaces
)

set(dependencies
  rclcpp
  communication_interfaces
)

ament_target_dependencies(${executable_door_manipulator_sim}
  ${dependencies}
)

target_link_libraries(${executable_door_manipulator_sim} ${library_door_manipulator_sim})


ament_target_dependencies(${library_door_manipulator_sim}
  ${dependencies}
)

target_include_directories(door_manipulator_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  ${library_door_manipulator_sim}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${executable_door_manipulator_sim}
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
