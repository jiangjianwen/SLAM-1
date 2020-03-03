cmake_minimum_required(VERSION 3.15)
project(optical_flow)

set(CMAKE_CXX_STANDARD 14)

# opencv
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})
# eigen
include_directories("/usr/include/eigen3")

add_executable(optical_flow main.cpp)
target_link_libraries(optical_flow ${OpenCV_LIBS} )


