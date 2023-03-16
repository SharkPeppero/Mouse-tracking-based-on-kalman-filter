cmake_minimum_required(VERSION 2.8)
project(mouse_track)

set(CMAKE_BUILD_TYPE "Debug")
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_DIRS} )

add_executable( mouse_track_node  main.cpp )
# 与Ceres和OpenCV链接
target_link_libraries( mouse_track_node  ${OpenCV_LIBS} )