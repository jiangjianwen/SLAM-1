cmake_minimum_required(VERSION 3.15)
project(ceresCurveFitting2)

set(CMAKE_CXX_STANDARD 14)

find_package( Ceres REQUIRED)
include_directories( ${CERES_INCLUDE_DIRS} )

add_executable(ceresCurveFitting2 main.cpp)
target_link_libraries(ceresCurveFitting2  ${CERES_LIBRARIES} )

