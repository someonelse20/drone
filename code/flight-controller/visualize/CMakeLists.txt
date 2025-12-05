cmake_minimum_required(VERSION 3.10)

project(get_data CXX)

add_library(get_data SHARED get_data.h get_data.cpp)

add_executable(ctest ctest.cpp)

target_link_libraries(ctest get_data)
target_link_libraries(get_data PRIVATE lsm9ds1)# ahrs PUBLIC stdc++)

