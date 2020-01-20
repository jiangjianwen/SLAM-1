cmake_minimum_required(VERSION 3.15)
project(imageBasics)

set(CMAKE_CXX_STANDARD 14)

#寻找Opencv库
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(imageBasics main.cpp)
target_link_libraries(imageBasics ${OpenCV_LIBS})