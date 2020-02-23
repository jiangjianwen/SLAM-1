cmake_minimum_required(VERSION 3.15)
project(HomographyMatrix)

set(CMAKE_CXX_STANDARD 14)

find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(HomographyMatrix main.cpp)
target_link_libraries(HomographyMatrix ${OpenCV_LIBS})

