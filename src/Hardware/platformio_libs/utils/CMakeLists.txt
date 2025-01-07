cmake_minimum_required(VERSION 3.8)
project(utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies. No idea if this is the right way to do it, feel free to tell me a better way
set(motor_dir ${CMAKE_SOURCE_DIR}/../motor)
set(drawer_dir ${CMAKE_SOURCE_DIR}/../drawer)
set(tray_manager_dir ${CMAKE_SOURCE_DIR}/../tray)
set(watchdog_dir ${CMAKE_SOURCE_DIR}/../watchdog)
set(module_config_dir ${CMAKE_SOURCE_DIR}/../../cpp/libs/module_config)
set(debug_dir ${CMAKE_SOURCE_DIR}/../debug)
set(looging_dir ${CMAKE_SOURCE_DIR}/../logging)
include_directories(include)
include_directories(${motor_dir}/include)
include_directories(${drawer_dir}/include)
include_directories(${tray_manager_dir}/include)
include_directories(${watchdog_dir}/include)
include_directories(${module_config_dir}/include)
include_directories(${debug_dir}/include)
include_directories(${looging_dir}/include)

add_library(library_config_manager STATIC src/config_manager.cpp)
add_library(library_motor_config STATIC ${motor_dir}/src/motor_config.cpp)
add_library(library_encoder_config STATIC ${motor_dir}/src/encoder_config.cpp)
add_library(library_motor_monitor_config STATIC ${motor_dir}/src/motor_monitor_config.cpp)
add_library(library_drawer_config STATIC ${drawer_dir}/src/electrical_drawer_config.cpp)
add_library(library_tray_manager_config STATIC ${tray_manager_dir}/src/tray_manager_config.cpp)
add_library(library_heartbeat_config STATIC ${watchdog_dir}/src/heartbeat_config.cpp)
add_library(library_rotating_file_handler_config STATIC ${looging_dir}/src/rotating_file_handler_config.cpp)

if(BUILD_TESTING)

  include(FetchContent)

  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.6.0
  )
  FetchContent_MakeAvailable(Catch2)

  add_executable(test_config_manager test/test_config_manager.cpp)

  add_dependencies(test_config_manager
    library_config_manager
    library_motor_config
    library_encoder_config
    library_motor_monitor_config
    library_drawer_config
    library_tray_manager_config
    library_heartbeat_config
    library_rotating_file_handler_config
  )

  target_link_libraries(test_config_manager
    Catch2::Catch2WithMain
    library_config_manager
    library_motor_config
    library_encoder_config
    library_motor_monitor_config
    library_drawer_config
    library_tray_manager_config
    library_heartbeat_config
    library_rotating_file_handler_config
  )

  target_compile_definitions(test_config_manager PRIVATE RUNNING_TESTS=true)
  target_compile_definitions(library_config_manager PRIVATE RUNNING_TESTS=true)
  target_compile_definitions(library_motor_config PRIVATE RUNNING_TESTS=true)
  target_compile_definitions(library_encoder_config PRIVATE RUNNING_TESTS=true)
  target_compile_definitions(library_motor_monitor_config PRIVATE RUNNING_TESTS=true)
  target_compile_definitions(library_drawer_config PRIVATE RUNNING_TESTS=true)
  target_compile_definitions(library_tray_manager_config PRIVATE RUNNING_TESTS=true)
  target_compile_definitions(library_heartbeat_config PRIVATE RUNNING_TESTS=true)
  target_compile_definitions(library_rotating_file_handler_config PRIVATE RUNNING_TESTS=true)

  # Register tests
  include(CTest) # Enables testing
  include(Catch)
  catch_discover_tests(test_config_manager)

endif()

install(TARGETS
    library_motor_config
    library_encoder_config
    library_motor_monitor_config
    library_drawer_config
    library_tray_manager_config
    library_heartbeat_config
    library_rotating_file_handler_config
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
