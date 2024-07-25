cmake_minimum_required(VERSION 3.5)
project(can)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

add_library(library_can SHARED
    ${CMAKE_CURRENT_LIST_DIR}/src/can_frame.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/can_helper.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/can_message.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/can_signal.cpp
)

target_include_directories(library_can
    PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
    $<INSTALL_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
) 

if(BUILD_TESTING)   
    include(FetchContent)

    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        v3.6.0 
    )
    FetchContent_MakeAvailable(Catch2)
    
    add_executable(
        can_test test/tests_robast_can_msgs.cpp ${unit_test_srcs}
    )

    target_link_libraries(can_test
        library_can
        Catch2::Catch2
        Catch2::Catch2WithMain
    )

    ament_target_dependencies(can_test ${dependencies})
    include(Catch)
    catch_discover_tests(can_test)

    target_include_directories(can_test
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

install(TARGETS
    library_can
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include/  
)

ament_export_include_directories(include)

ament_export_libraries(library_can)

ament_package()