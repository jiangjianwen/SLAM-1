cmake_minimum_required(VERSION 3.15)
project(E2Rt)

set(CMAKE_CXX_STANDARD 14)
include_directories("/usr/include/eigen3")
# 使用Sophus 需要加上find_package
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(E2Rt main.cpp)
target_link_libraries(E2Rt ${Sophus_LIBRARIES})

