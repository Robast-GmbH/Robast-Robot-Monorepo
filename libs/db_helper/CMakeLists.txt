cmake_minimum_required(VERSION 3.5)
project(db_helper)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

add_library(library_db_helper SHARED
    ${CMAKE_CURRENT_LIST_DIR}/src/postgresql_connector.cpp
)

target_include_directories(library_db_helper
    PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
    $<INSTALL_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
) 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lpqxx")

set(PQXX /usr/local/include/pqxx)

find_library(PQXX_LIB pqxx)

install(TARGETS
    library_db_helper
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include/  
)

ament_export_include_directories(include)

ament_export_libraries(library_db_helper)

ament_package()
TARGET_LINK_LIBRARIES(library_db_helper  ${PQXX_LIB})