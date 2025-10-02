cmake_minimum_required(VERSION 3.6)

project(flight_controller)

add_library(ahrs ./ahrs/ahrs.cpp ./ahrs/ahrs.h)

add_executable(test test.cpp)

