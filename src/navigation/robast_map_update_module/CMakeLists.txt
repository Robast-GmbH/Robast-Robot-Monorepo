cmake_minimum_required(VERSION 3.8)
project(robast_map_update_module)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robast_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(OpenCV REQUIRED)
if("${OpenCV_VERSION}" VERSION_LESS "3.0")
message(FATAL_ERROR "This package needs OpenCV >= 3.0")
endif()
if("${OpenCV_VERSION}" VERSION_LESS "4.0")
message(WARNING "This package supports OpenCV 3, but some features may not be "
"available. Upgrade to OpenCV 4 to take advantage of all features.")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(DEPENDENCIES
  rclcpp
  geometry_msgs
  image_geometry
  map_msgs
  nav_msgs
  tf2_geometry_msgs
  OpenCV
)

include_directories(
  # ${Boost_INCLUDE_DIRS}
  # ${OpenCV_INCLUDE_DIRS}
  include
)

install(
  DIRECTORY include/robast_map_update_module/
  DESTINATION include/robast_map_update_module/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

add_library(combine_grids STATIC
src/combine_grids/grid_compositor.cpp
src/combine_grids/grid_warper.cpp
src/combine_grids/merging_pipeline.cpp
)

add_executable(map_combine_node
  src/map_combine_node.cpp
)


#############
## Install ##
#############
target_include_directories(map_combine_node PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_include_directories(combine_grids PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(combine_grids ${rclcpp_LIBRARIES} ${OpenCV_LIBS})

target_link_libraries(map_combine_node combine_grids)

ament_target_dependencies(map_combine_node ${DEPENDENCIES})

install(
  TARGETS combine_grids
  EXPORT export_combine_grids
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS map_combine_node
  DESTINATION lib/${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

ament_export_include_directories(include)
ament_export_libraries(combine_grids)
ament_package()