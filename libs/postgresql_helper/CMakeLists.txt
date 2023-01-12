cmake_minimum_required(VERSION 3.5)
project(postgresql_helper)

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

add_library(library_postgresql_helper SHARED
    ${CMAKE_CURRENT_LIST_DIR}/src/postgresql_helper.cpp
)

target_include_directories(library_postgresql_helper
    PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
    $<INSTALL_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
) 

install(TARGETS
    library_serial_helper
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include/  
)

ament_export_include_directories(include)

ament_export_libraries(library_postgresql_helper)

ament_package()