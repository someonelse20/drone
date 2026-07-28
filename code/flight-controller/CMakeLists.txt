cmake_minimum_required(VERSION 3.10)

project(flight_controller CXX)

add_library(lsm9ds1_custom lsm9ds1.h lsm9ds1_reg.h lsm9ds1.cpp)
add_library(motor motor.h motor.cpp)
add_library(ahrs ahrs/ahrs.h ahrs/ahrs.cpp)
add_library(pid pid.h pid.cpp)

add_executable(flight_controller main.cpp)
# add_executable(i2c lsm9ds1.h lsm9ds1.cpp)

target_link_libraries(flight_controller PRIVATE lsm9ds1_custom motor ahrs pid PUBLIC stdc++)
target_link_libraries(lsm9ds1_custom PRIVATE pigpio PUBLIC rt pthread)
target_link_libraries(motor PRIVATE pigpio PUBLIC rt pthread)

set(CMAKE_BUILD_TYPE=Debug)

