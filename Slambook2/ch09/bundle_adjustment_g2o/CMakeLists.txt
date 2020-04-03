cmake_minimum_required(VERSION 3.15)
project(bundle_adjustment_g2o)

set(CMAKE_CXX_STANDARD 14)


include_directories(${PROJECT_SOURCE_DIR} )

include_directories("/usr/include/eigen3")

find_package( Ceres REQUIRED)
include_directories( ${CERES_INCLUDE_DIRS} )

add_library(bal_common common.cpp)

add_executable(bundle_adjustment_g2o main.cpp)
target_link_libraries( bundle_adjustment_g2o ${CERES_LIBRARIES} bal_common )
