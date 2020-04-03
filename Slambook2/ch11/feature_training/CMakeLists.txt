cmake_minimum_required(VERSION 3.15)
project(feature_training)

set(CMAKE_CXX_STANDARD 14)

# dbow3
# dbow3 is a simple lib so I assume you installed it in default directory
set( DBoW3_INCLUDE_DIRS "/usr/local/include" )
set( DBoW3_LIBS "/usr/local/lib/libDBoW3.so" )

#寻找Opencv库
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(feature_training main.cpp)
target_link_libraries( feature_training ${OpenCV_LIBS} ${DBoW3_LIBS} )
