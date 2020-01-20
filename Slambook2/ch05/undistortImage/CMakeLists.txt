cmake_minimum_required(VERSION 3.15)
project(undistortImage)

set(CMAKE_CXX_STANDARD 14)

include_directories("/usr/include/eigen3")
#寻找Opencv库
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})


find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
add_executable(undistortImage main.cpp)
target_link_libraries(undistortImage ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES} )
