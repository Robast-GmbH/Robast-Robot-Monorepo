cmake_minimum_required(VERSION 3.5)
project(qr_code_scanner)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(communication_interfaces REQUIRED)

include_directories(include/qr_code_scanner)
include_directories(include/quirc/lib)
include_directories(${OpenCV_INCLUDE_DIRS})

# Create a library from the source code
add_library(qr_code_scanner_lib STATIC src/qr_code_scanner.cpp
include/quirc/lib/decode.c
include/quirc/lib/identify.c
include/quirc/lib/quirc.c
include/quirc/lib/version_db.c
)

# Link the library to rclcpp and other dependencies
ament_target_dependencies(qr_code_scanner_lib rclcpp sensor_msgs OpenCV communication_interfaces)

# Create an executable for the node, which will link against the library
add_executable(qr_code_scanner src/main.cpp)
target_link_libraries(qr_code_scanner qr_code_scanner_lib)


# Install the executable
install(TARGETS
  qr_code_scanner
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
      test_qr_code_scanner test/test_qr_code_scanner.cpp
  )

  target_link_libraries(test_qr_code_scanner
      Catch2::Catch2
      Catch2::Catch2WithMain
      qr_code_scanner_lib
  )

  set(test_dependencies
  geometry_msgs
  rclcpp
  )

  ament_target_dependencies(test_qr_code_scanner ${test_dependencies})
  include(Catch)
  catch_discover_tests(test_qr_code_scanner)

  target_include_directories(test_qr_code_scanner
      PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/>
  )

  include(CTest)
endif()
# Export dependencies
ament_package()
