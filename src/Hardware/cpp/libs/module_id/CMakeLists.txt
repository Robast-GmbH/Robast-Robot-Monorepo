cmake_minimum_required(VERSION 3.5)
project(module_id)

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
add_library(library_module_id INTERFACE)

# Specify include directories for the header-only library
target_include_directories(library_module_id
    INTERFACE 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
    $<INSTALL_INTERFACE:include>
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
        module_id_test test/test_module_id.cpp
    )

    target_link_libraries(module_id_test
        library_module_id
        Catch2::Catch2
        Catch2::Catch2WithMain
    )

    ament_target_dependencies(module_id_test ${dependencies})
    include(Catch)
    catch_discover_tests(module_id_test)

    target_include_directories(module_id_test
        PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/>
    )

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

ament_package()