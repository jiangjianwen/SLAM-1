cmake_minimum_required(VERSION 3.15)
project(trajectoryError)

set(CMAKE_CXX_STANDARD 14)
include_directories("/usr/include/eigen3")
# 使用Sophus 需要加上find_package
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})


add_executable(trajectoryError main.cpp)
target_link_libraries(trajectoryError ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES} )

