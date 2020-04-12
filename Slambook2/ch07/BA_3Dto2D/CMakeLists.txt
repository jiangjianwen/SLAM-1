cmake_minimum_required(VERSION 3.15)
project(BA_3Dto2D)

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

# find_package( CSparse REQUIRED )

#

add_executable(BA_3Dto2D main.cpp)
target_link_libraries(BA_3Dto2D ${OpenCV_LIBS} g2o_core g2o_stuff g2o_types_sba  )

