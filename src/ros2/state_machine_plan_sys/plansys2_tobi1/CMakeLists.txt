cmake_minimum_required(VERSION 3.8)
project(plansys2_tobi1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_bt_actions REQUIRED)
find_package(plansys2_bringup REQUIRED)
find_package(plansys2_terminal REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(communication_interfaces REQUIRED)

set(dependencies 
  rclcpp
  rclcpp_action
  plansys2_msgs
  plansys2_domain_expert
  plansys2_executor
  plansys2_planner
  plansys2_problem_expert
  plansys2_pddl_parser
  jsoncpp
  communication_interfaces
)

include_directories(include)
# target_include_directories(led_colors PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/plansys2_tobi1/led_colors.hpp)
add_executable(drawer_open_action_node src/drawer_open_action_node.cpp )
ament_target_dependencies(drawer_open_action_node ${dependencies})

target_link_libraries(drawer_open_action_node jsoncpp_lib)

# target_link_libraries(drawer_open_action_node PRIVATE communication_interfaces::led_colors)

# target_include_directories(led_colors PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/plansys2_tobi1>
#   $<INSTALL_INTERFACE:include/plansys2_tobi1>
# )

target_include_directories(drawer_open_action_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(DIRECTORY
  # launch
  pddl
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  drawer_open_action_node  
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()