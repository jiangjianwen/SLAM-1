cmake_minimum_required(VERSION 3.15)
project(ceresCurveFitting)

set(CMAKE_CXX_STANDARD 14)

find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

find_package( Ceres REQUIRED)
include_directories( ${CERES_INCLUDE_DIRS} )

add_executable(ceresCurveFitting main.cpp)
target_link_libraries(ceresCurveFitting ${OpenCV_LIBS} ${CERES_LIBRARIES} )

