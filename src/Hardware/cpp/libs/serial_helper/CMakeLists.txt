cmake_minimum_required(VERSION 3.5)
project(serial_helper)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

add_library(library_serial_helper SHARED
    ${CMAKE_CURRENT_LIST_DIR}/src/serial_helper.cpp
)

target_include_directories(library_serial_helper
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

ament_export_libraries(library_serial_helper)

ament_package()