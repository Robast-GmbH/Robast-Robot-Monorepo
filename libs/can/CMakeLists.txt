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
find_package(ament_cmake_catch2 REQUIRED)

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
    file(GLOB_RECURSE unit_test_srcs "test/*.cpp")
    
    ament_add_catch2(
        can_test test/tests_main.cpp ${unit_test_srcs}
    )

    target_link_libraries(can_test
        library_can
    )

    target_include_directories(can_test
        PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/>
    )
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