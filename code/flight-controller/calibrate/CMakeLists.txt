cmake_minimum_required(VERSION 3.50)

project(get_data)

add_executable(get_data "./get_data.cpp")

target_link_libraries(get_data PRIVATE wiringPi PUBLIC stdc++)
target_link_libraries(get_data PRIVATE lsm9ds1 PUBLIC stdc++)

