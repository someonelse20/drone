cmake_minimum_required(VERSION 3.10)

project(ahrs_test CXX)

add_library(gen_data gen_data.h gen_data.cpp)
add_library(ahrs ../ahrs.h ../ahrs.cpp)

add_executable(main ahrs_test.cpp)

target_link_libraries(main ahrs gen_data)

set(CMAKE_BUILD_TYPE=Debug)

