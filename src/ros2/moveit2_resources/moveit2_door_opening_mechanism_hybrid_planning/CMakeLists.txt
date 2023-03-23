cmake_minimum_required(VERSION 3.8)
project(moveit2_door_opening_mechanism_hybrid_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_hybrid_planning REQUIRED)


set(executable_hybrid_planning_demo_node hybrid_planning_demo_node)

add_executable(${executable_hybrid_planning_demo_node} src/hybrid_planning_demo_node.cpp)

set(dependencies
  rclcpp
  moveit_hybrid_planning
)

ament_target_dependencies(${executable_hybrid_planning_demo_node}
  ${dependencies}
)


if(BUILD_TESTING)
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()
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
  ${executable_hybrid_planning_demo_node}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
