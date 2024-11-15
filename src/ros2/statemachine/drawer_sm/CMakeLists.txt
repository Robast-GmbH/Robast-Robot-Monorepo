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
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(communication_interfaces REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(bt_plugins REQUIRED)
find_package(bt_base_nodes REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  communication_interfaces
  behaviortree_cpp
  bt_plugins
  bt_base_nodes
)

include_directories(
  include
)

add_executable(drawer_sm_startup_debug src/drawer_sm_startup_debug.cpp)

option(BTCPP_ENABLE_COROUTINES "Enable boost coroutines" ON)

#---- Include boost to add coroutines ----
if(BTCPP_ENABLE_COROUTINES)
    find_package(Boost COMPONENTS coroutine QUIET)

    if(Boost_FOUND)
        string(REPLACE "." "0" Boost_VERSION_NODOT ${Boost_VERSION})
        if(NOT Boost_VERSION_NODOT VERSION_LESS 105900)
            message(STATUS "Found boost::coroutine2.")
            add_definitions(-DBT_BOOST_COROUTINE2)
            set(BT_COROUTINES_FOUND true)
        elseif(NOT Boost_VERSION_NODOT VERSION_LESS 105300)
            message(STATUS "Found boost::coroutine.")
            add_definitions(-DBT_BOOST_COROUTINE)
            set(BT_COROUTINES_FOUND true)
        endif()
        include_directories(${Boost_INCLUDE_DIRS})
    endif()

    if(NOT DEFINED BT_COROUTINES_FOUND)
        message(STATUS "Boost coroutines disabled. Install Boost (version 1.59+ recommended).")
    endif()
else()
    message(STATUS "Boost coroutines disabled by CMake option.")
endif()

set(executable_heartbeat_tree_spawner heartbeat_tree_spawner)

add_executable(${executable_heartbeat_tree_spawner}
  src/heartbeat_tree_spawner_main.cpp)

set(library_heartbeat_tree_spawner ${executable_heartbeat_tree_spawner}_core)

add_library(${library_heartbeat_tree_spawner} SHARED
  src/heartbeat_tree_spawner.cpp
)

ament_target_dependencies(${executable_heartbeat_tree_spawner}
  ${dependencies}
)
target_link_libraries(${executable_heartbeat_tree_spawner} ${library_heartbeat_tree_spawner})

ament_target_dependencies(${library_heartbeat_tree_spawner}
  ${dependencies}
)

target_include_directories(${executable_heartbeat_tree_spawner} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  drawer_sm_startup_debug
  ${dependencies}
)

target_include_directories(drawer_sm_startup_debug
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS
drawer_sm_startup_debug
${executable_heartbeat_tree_spawner}
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
config
trees
DESTINATION share/${PROJECT_NAME})


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
endif()


ament_export_include_directories(include)

ament_package()

