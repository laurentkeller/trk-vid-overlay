cmake_minimum_required(VERSION 2.8)

project(trkVidOL)
set (CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)

add_compile_definitions(ARTAG_225)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("/home/matthias/Documents/Code/tpppl/inc")
link_directories("/home/matthias/Documents/Code/tpppl/build/lib")

add_executable(test testopencv.cpp)
target_link_libraries(test ${OpenCV_LIBS})

add_executable(trkVidOL trkVidOL.cpp)
target_link_libraries(trkVidOL ${OpenCV_LIBS} datfile tags3 exception)
