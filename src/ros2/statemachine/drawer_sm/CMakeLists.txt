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

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  communication_interfaces
  behaviortree_cpp
  bt_plugins
)

include_directories(
  include
)

add_executable(drawer_sm2 src/drawer_sm.cpp)

ament_target_dependencies(drawer_sm2
  ${dependencies}
)

# target_link_libraries(drawe_sm2 ${dependencies})

target_include_directories(drawer_sm2
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS
drawer_sm2
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
trees
DESTINATION trees)

install(DIRECTORY
include/
DESTINATION 
include/)


ament_export_include_directories(include)
# ament_export_libraries(${lib_name})
# ament_export_dependencies(${dependencies})

ament_package()





    #old26+:
# include_directories(
#   include
# )

# set(lib_name bt_engine)

# add_executable(drawer_sm2 src/drawer_sm.cpp)
# add_executable(drawer_sm src/drawer_sm.cpp)

# ament_target_dependencies(drawer_sm2
# # ament_target_dependencies(drawer_sm
# #   ${dependencies}
# # )

# add_library(${lib_name} SHARED
#   src/behavior_tree_engine.cpp
# )

# ament_target_dependencies(${lib_name} drawer_sm
#   ${dependencies}
# )

# # target_link_libraries(drawe_sm2 ${dependencies})


# ament_package()
# install(TARGETS
# drawer_sm2
#   DESTINATION lib/${PROJECT_NAME})
#   drawer_sm
#   DESTINATION lib/${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
#   )

# install(DIRECTORY
#   trees/
#   include/
#   DESTINATION trees/
#   include/)

# ament_export_include_directories(include)
# ament_export_libraries(${lib_name})
# ament_export_dependencies(${dependencies})

#   install(DIRECTORY
#   trees
#     DESTINATION trees)
# ament_package()
