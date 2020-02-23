cmake_minimum_required(VERSION 3.15)
project(pose_estimation_2d2d)

set(CMAKE_CXX_STANDARD 14)

find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(pose_estimation_2d2d main.cpp)
target_link_libraries(pose_estimation_2d2d ${OpenCV_LIBS})