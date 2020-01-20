cmake_minimum_required(VERSION 3.15)
project(plotTrajectory)

set(CMAKE_CXX_STANDARD 14)
include_directories("/usr/include/eigen3")
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(plotTrajectory main.cpp)
target_link_libraries(plotTrajectory ${Pangolin_LIBRARIES})