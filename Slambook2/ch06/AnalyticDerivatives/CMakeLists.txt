cmake_minimum_required(VERSION 3.15)
project(useCeres)

set(CMAKE_CXX_STANDARD 14)

find_package( Ceres REQUIRED)
include_directories( ${CERES_INCLUDE_DIRS} )

#add_executable(NumericDerivatives NumericDerivatives.cpp AnalyticDerivatives.cpp)
#target_link_libraries(NumericDerivatives  ${CERES_LIBRARIES} )

add_executable(AnalyticDerivatives AnalyticDerivatives.cpp)
target_link_libraries(AnalyticDerivatives  ${CERES_LIBRARIES} )


