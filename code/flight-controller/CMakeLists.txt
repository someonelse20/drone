cmake_minimum_required(VERSION 3.10)

project(flight_controller CXX)

add_library(ahrs ahrs.h ahrs.cpp)
add_library(motor motor.h motor.cpp)

add_executable(flight_controller main.cpp)
add_executable(i2c lsm9ds1.h lsm9ds1.cpp)

target_link_libraries(flight_controller PRIVATE lsm9ds1 ahrs PUBLIC stdc++)
target_link_libraries(motor PRIVATE pigpio PUBLIC rt pthread)
target_link_libraries(i2c PRIVATE pigpio PUBLIC rt pthread)

set(CMAKE_BUILD_TYPE=Debug)

