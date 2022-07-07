cmake_minimum_required(VERSION 3.8)
project(robast_serial)# VERSION 1.0.1 DESCRIPTION "implementaton of our serial controller ")

set(CMAKE_CXX_STANDARD 11)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
add_library(robast_serial STATIC src/robast_serial.cpp)

install(TARGETS robast_serial DESTINATION /usr/lib)
install(FILES include/robast_serial.h DESTINATION include)