cmake_minimum_required(VERSION 3.00)

project(lsm9ds1_test)

#add_executable(lsm9ds1_test "test.cpp")
add_executable(lsm9ds1_test "./calibrate/get_data.cpp")

target_link_libraries(lsm9ds1_test PRIVATE wiringPi PUBLIC stdc++)
target_link_libraries(lsm9ds1_test PRIVATE lsm9ds1 PUBLIC stdc++)

