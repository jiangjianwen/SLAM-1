cmake_minimum_required(VERSION 3.15)
project(pose_graph_g2o_lie_algebra)

set(CMAKE_CXX_STANDARD 14)

# Eigen
include_directories("/usr/include/eigen3")

# g2o
find_package(g2o REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(pose_graph_g2o_lie_algebra main.cpp)
target_link_libraries(pose_graph_g2o_lie_algebra ${Sophus_LIBRARIES} g2o_core g2o_stuff g2o_types_slam3d )
