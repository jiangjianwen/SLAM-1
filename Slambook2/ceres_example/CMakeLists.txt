cmake_minimum_required(VERSION 3.15)
project(ceres_example)

set(CMAKE_CXX_STANDARD 14)

find_package( Ceres REQUIRED)
include_directories( ${CERES_INCLUDE_DIRS} )

add_executable(helloworld helloworld.cpp )
target_link_libraries(helloworld ${CERES_LIBRARIES} )


add_executable(helloworld_numeric_diff helloworld_numeric_diff.cpp )
target_link_libraries(helloworld_numeric_diff ${CERES_LIBRARIES} )


