cmake_minimum_required(VERSION 3.10)

project(ahrs_test CXX)

add_library(ahrs ../ahrs/ahrs.h ../ahrs/ahrs.cpp ../ahrs/ahrs_math.cpp)

add_executable(main ahrs_test.h ahrs_test.cpp)

target_link_libraries(main ahrs)

set(CMAKE_BUILD_TYPE=Debug)

