cmake_minimum_required(VERSION 3.8)
project(static_layer_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(map_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)



include_directories(include)
  
#add_definitions(${EIGEN3_DEFINITIONS})
  

set(dependencies
map_msgs
message_filters
nav2_costmap_2d
nav2_msgs
nav_msgs
pluginlib
rclcpp
#rclcpp_lifecycle
tf2
tf2_geometry_msgs
)

add_library(layers SHARED
src/static_layer_feuerplan.cpp)

ament_target_dependencies(layers
  ${dependencies}
)
target_link_libraries(layers)
#target_compile_definitions(layers PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(TARGETS layers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
#ament_export_targets(
#  export_${PROJECT_NAME}
#)
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)
ament_package()
