cmake_minimum_required(VERSION 3.00)

project(flight_controller)

add_executable(flight_controller "simple_test.cpp")

add_subdirectory("Fusion/Fusion")

add_library(driver "driver.h")

set_target_properties(driver PROPERTIES LINKER_LANGUAGE CXX)

#target_link_libraries(flight_controller PRIVATE Fusion)

target_link_libraries(driver PRIVATE wiringPi PUBLIC stdc++)
target_link_libraries(driver PRIVATE lsm9ds1 PUBLIC stdc++)
target_link_libraries(flight_controller PRIVATE lsm9ds1 PUBLIC stdc++)

