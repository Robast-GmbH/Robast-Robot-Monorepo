cmake_minimum_required(VERSION 3.10.2)
project(drawer_sm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(communication_interfaces REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(bt_plugins REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  communication_interfaces
  behaviortree_cpp_v3
  bt_plugins
)

add_executable(drawer_sm2 src/drawer_sm.cpp)

ament_target_dependencies(drawer_sm2
  ${dependencies}
)

# target_link_libraries(drawe_sm2 ${dependencies})


ament_package()
install(TARGETS
drawer_sm2
  DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY
  trees
    DESTINATION trees)