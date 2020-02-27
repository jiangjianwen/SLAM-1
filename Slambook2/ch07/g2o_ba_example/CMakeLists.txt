cmake_minimum_required(VERSION 3.15)
project(g2o_ba_example)

set(CMAKE_CXX_STANDARD 14)

# eigen
include_directories("/usr/include/eigen3")
# opencv
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

# g2o
find_package(g2o REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

add_executable(g2o_ba_example main.cpp)
target_link_libraries(g2o_ba_example ${OpenCV_LIBS} g2o_core g2o_stuff )




