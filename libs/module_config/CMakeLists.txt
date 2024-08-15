cmake_minimum_required(VERSION 3.5)
project(module_config)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# Define the header-only library
add_library(library_module_config INTERFACE)

# Specify include directories for the header-only library
target_include_directories(library_module_config
    INTERFACE 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
    $<INSTALL_INTERFACE:include>
) 

if(BUILD_TESTING)
    include(CTest)

    find_package(ament_cmake_flake8 REQUIRED)
    ament_flake8()
    find_package(ament_cmake_xmllint REQUIRED)
    ament_xmllint()
    find_package(ament_cmake_pep257 REQUIRED)
    ament_pep257()
    find_package(ament_cmake_cppcheck REQUIRED)
    ament_cppcheck()
endif()

install(DIRECTORY include/
    DESTINATION include
)

ament_export_include_directories(include)

ament_export_libraries(library_module_config)

ament_package()