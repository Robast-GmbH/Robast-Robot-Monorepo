cmake_minimum_required(VERSION 3.8)
project(bt_plugins)

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
find_package(behaviortree_cpp REQUIRED)
find_package(error_utils REQUIRED)
find_package(bt_base_types REQUIRED)
find_package(robotnik_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)


set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  communication_interfaces
  behaviortree_cpp
  error_utils
  bt_base_types
  nav_msgs
  robotnik_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
include_directories(
  include
)

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/behavior_tree_engine.cpp
)
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

ament_target_dependencies(${library_name}
  ${dependencies}
)


add_library(initialize_led_vector_action_bt_node SHARED src/action/initialize_led_vector_action.cpp)
list(APPEND plugin_libs initialize_led_vector_action_bt_node)

# add_library(change_led_action_bt_node SHARED src/action/change_led_action.cpp)
# list(APPEND plugin_libs change_led_action_bt_node)

add_library(drawer_change_state_request_action_bt_node SHARED src/action/drawer_change_state_request_action.cpp)
list(APPEND plugin_libs drawer_change_state_request_action_bt_node)

add_library(open_drawer_action_bt_node SHARED src/action/open_drawer_action.cpp)
list(APPEND plugin_libs open_drawer_action_bt_node)

add_library(move_electric_drawer_action_bt_node SHARED src/action/move_electric_drawer_action.cpp)
list(APPEND plugin_libs move_electric_drawer_action_bt_node)

add_library(drawer_status_condition_bt_node SHARED src/condition/drawer_status_condition.cpp)
list(APPEND plugin_libs drawer_status_condition_bt_node)

add_library(nfc_to_drawer_action_bt_node SHARED src/action/nfc_to_drawer_action.cpp)
list(APPEND plugin_libs nfc_to_drawer_action_bt_node)

add_library(reset_decorator_bt_node SHARED src/decorator/reset_tree_decorator.cpp)
list(APPEND plugin_libs reset_decorator_bt_node)

add_library(electric_drawer_status_condition_bt_node SHARED src/condition/electric_drawer_status_condition.cpp)
list(APPEND plugin_libs electric_drawer_status_condition_bt_node)

add_library(base_error_decorator_node SHARED src/decorator/base_error_decorator.cpp)
list(APPEND plugin_libs base_error_decorator_node)

add_library(partial_colorize_leds_action_bt_node SHARED src/action/partial_colorize_leds_action.cpp)
list(APPEND plugin_libs partial_colorize_leds_action_bt_node)

add_library(evaluate_drive_direction_action_bt_node SHARED src/action/evaluate_drive_direction_action.cpp)
list(APPEND plugin_libs evaluate_drive_direction_action_bt_node)

add_library(publish_led_action_bt_node SHARED src/action/publish_led_action.cpp)
list(APPEND plugin_libs publish_led_action_bt_node)

add_library(get_blackboard_value_action_bt_node SHARED src/action/get_blackboard_value_action.cpp)
list(APPEND plugin_libs get_blackboard_value_action_bt_node)

add_library(publish_drawer_status_action_bt_node SHARED src/action/publish_drawer_status_action.cpp)
list(APPEND plugin_libs publish_drawer_status_action_bt_node)

add_library(bool_topic_condition_node SHARED src/condition/bool_topic_condition.cpp)
list(APPEND plugin_libs bool_topic_condition_node)

add_library(led_changed_condition_node SHARED src/condition/led_changed_condition.cpp)
list(APPEND plugin_libs led_changed_condition_node)

add_library(is_robot_charging_condition_node SHARED src/condition/is_robot_charging_condition.cpp)
list(APPEND plugin_libs is_robot_charging_condition_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${library_name}
                ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES xml/bt_nodes.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  include(FetchContent)

  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        ab6c7375be9a8e71ee84c6f8537113f9f47daf99 # should equal tag v3.2.1
  )
  FetchContent_MakeAvailable(Catch2)


  include_directories(tests/*)
  add_library(test_helper SHARED tests/test_helper.cpp)
  list(APPEND test_libs test_helper)

  # add_executable(led_test tests/actions/change_led_action_test.cpp)
  # list(APPEND test_executables led_test)
  add_executable(open_drawer_test tests/actions/open_drawer_action_test.cpp)
  list(APPEND test_executables open_drawer_test)
  add_executable(drawer_status_condition_test tests/conditions/drawer_status_condition_test.cpp)
  list(APPEND test_executables drawer_status_condition_test)
  add_executable(nfc_to_drawer_action_test tests/actions/nfc_to_drawer_action_test.cpp)
  list(APPEND test_executables nfc_to_drawer_action_test)
  add_executable(move_electric_drawer_action_test tests/actions/move_electric_drawer_action_test.cpp)
  list(APPEND test_executables move_electric_drawer_action_test)
  add_executable(error_decorator_test tests/decorators/error_decorator_test.cpp)
  list(APPEND test_executables error_decorator_test)
  add_executable(calculate_direction_test tests/calculate_direction_test.cpp)
  list(APPEND test_executables calculate_direction_test)
  

  ament_target_dependencies(${test_libs}
  ${dependencies})

  foreach(test ${test_executables})
    target_link_libraries(${test}
    ${library_name}
    ${plugin_libs}
    ${test_libs}
    Catch2::Catch2
    Catch2::Catch2WithMain
    )
    ament_target_dependencies(${test}
    ${dependencies})
      include(Catch)
    catch_discover_tests(${test})
  endforeach()
  
  include(CTest)

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

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${library_name}
  ${plugin_libs}
)

ament_export_dependencies(${dependencies})

ament_package()