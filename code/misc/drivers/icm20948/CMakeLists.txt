cmake_minimum_required(VERSION 3.00)

project(spi_test)

add_library(spi "./SPI.h")
add_library(registers "./registers.h")
add_executable(spi_test "./spi_test.cpp")

target_link_libraries(spi_test PRIVATE wiringPi PUBLIC stdc++)

set_target_properties(spi PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(registers PROPERTIES LINKER_LANGUAGE CXX)

