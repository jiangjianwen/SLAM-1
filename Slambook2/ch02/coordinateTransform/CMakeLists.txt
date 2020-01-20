cmake_minimum_required(VERSION 3.15)
project(coordinateTransform)

set(CMAKE_CXX_STANDARD 14)

include_directories("/usr/include/eigen3")

add_executable(coordinateTransform main.cpp)