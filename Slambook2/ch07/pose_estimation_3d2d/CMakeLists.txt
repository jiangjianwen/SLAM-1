cmake_minimum_required(VERSION 3.15)
project(pose_estimation_3d2d)

set(CMAKE_CXX_STANDARD 14)

# opencv
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})
# eigen
include_directories("/usr/include/eigen3")
# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
# g2o
# g2o
find_package(g2o REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

add_executable(pose_estimation_3d2d main.cpp)
target_link_libraries(pose_estimation_3d2d ${OpenCV_LIBS} g2o_core g2o_stuff )


