cmake_minimum_required(VERSION 3.15)
project(useSophus)

set(CMAKE_CXX_STANDARD 14)
include_directories("/usr/include/eigen3")
# 使用Sophus 需要加上find_package
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
add_executable(useSophus main.cpp)
target_link_libraries(useSophus ${Sophus_LIBRARIES})


