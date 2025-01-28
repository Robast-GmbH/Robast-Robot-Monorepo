cmake_minimum_required(VERSION 3.5)
project(initial_pose_publisher)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add the executable
add_executable(initial_pose_publisher src/initial_pose_publisher.cpp)

# Link libraries
ament_target_dependencies(initial_pose_publisher rclcpp geometry_msgs)

# Install the executable
install(TARGETS
  initial_pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  include(FetchContent)

  FetchContent_Declare(
      Catch2
      GIT_REPOSITORY https://github.com/catchorg/Catch2.git
      GIT_TAG        v3.8.0
  )
  FetchContent_MakeAvailable(Catch2)
  
  add_executable(
      test_initial_pose_publisher test/test_initial_pose_publisher.cpp
  )

  target_link_libraries(test_initial_pose_publisher
      Catch2::Catch2
      Catch2::Catch2WithMain
  )

  set(test_dependencies
  geometry_msgs
  rclcpp
  )

  ament_target_dependencies(test_initial_pose_publisher ${test_dependencies})
  include(Catch)
  catch_discover_tests(test_initial_pose_publisher)

  target_include_directories(test_initial_pose_publisher
      PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/>
  )

  include(CTest)
endif()
# Export dependencies
ament_package()
