cmake_minimum_required(VERSION 3.15)
project(gaussNewton)

set(CMAKE_CXX_STANDARD 14)

include_directories("/usr/include/eigen3")
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(gaussNewton main.cpp)
target_link_libraries(gaussNewton ${OpenCV_LIBS})
