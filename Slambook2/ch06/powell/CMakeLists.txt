cmake_minimum_required(VERSION 3.15)
project(powell)

set(CMAKE_CXX_STANDARD 14)

find_package( Ceres REQUIRED)
include_directories( ${CERES_INCLUDE_DIRS} )

add_executable(powell main.cpp)
target_link_libraries(powell  ${CERES_LIBRARIES} )

