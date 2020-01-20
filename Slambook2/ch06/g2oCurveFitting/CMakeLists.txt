cmake_minimum_required(VERSION 3.15)
project(g2oCurveFitting)

set(CMAKE_CXX_STANDARD 14)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# g2o
find_package(g2o REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# Eigen
include_directories("/usr/include/eigen3")



add_executable(g2oCurveFitting main.cpp)
target_link_libraries(g2oCurveFitting ${OpenCV_LIBS} ${CERES_LIBRARIES} g2o_core g2o_stuff )


