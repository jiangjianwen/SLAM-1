cmake_minimum_required(VERSION 3.15)
project(CeresBundleAdjustment)

set(CMAKE_CXX_STANDARD 14)

find_package( Ceres REQUIRED)
include_directories( ${CERES_INCLUDE_DIRS} )

add_executable(CeresBundleAdjustment main.cpp)
target_link_libraries(CeresBundleAdjustment  ${CERES_LIBRARIES} )
