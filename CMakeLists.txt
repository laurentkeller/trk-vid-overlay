cmake_minimum_required(VERSION 2.8)

project(trkVidOL)
set (CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(trkVidOL trkVidOL.cpp)
target_link_libraries(trkVidOL ${OpenCV_LIBS} atrkutil)
