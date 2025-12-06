cmake_minimum_required(VERSION 3.10)

project(get_data CXX)

add_library(get_data SHARED get_data.h get_data.cpp)
add_library(ahrs ../ahrs.h ../ahrs.cpp)

add_executable(gyro_test gyro_test_qat.cpp)

target_link_libraries(gyro_test PRIVATE get_data ahrs)
target_link_libraries(get_data PRIVATE lsm9ds1)# ahrs PUBLIC stdc++)

