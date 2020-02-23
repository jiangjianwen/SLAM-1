cmake_minimum_required(VERSION 3.15)
project(ORB_Self_ShenLan)

set(CMAKE_CXX_STANDARD 14)

find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(ORB_Self_ShenLan main.cpp)
target_link_libraries(ORB_Self_ShenLan ${OpenCV_LIBS})


