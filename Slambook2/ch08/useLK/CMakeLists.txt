cmake_minimum_required(VERSION 3.15)
project(useLK)

set(CMAKE_CXX_STANDARD 14)

# opencv
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(useLK main.cpp)
target_link_libraries(useLK ${OpenCV_LIBS} g2o_core g2o_stuff )

