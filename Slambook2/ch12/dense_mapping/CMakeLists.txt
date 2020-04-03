cmake_minimum_required(VERSION 3.15)
project(dense_mapping)

set(CMAKE_CXX_STANDARD 14)

# 使用Sophus 需要加上find_package
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")

#寻找Opencv库
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(dense_mapping main.cpp)
target_link_libraries(dense_mapping ${Sophus_LIBRARIES} ${OpenCV_LIBS} )
