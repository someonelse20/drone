cmake_minimum_required(VERSION 3.6)

project(flight_controller CXX)

add_library(ahrs ahrs.h ahrs.cpp)

add_executable(flight_controller main.cpp)

target_link_libraries(flight_controller ahrs)

