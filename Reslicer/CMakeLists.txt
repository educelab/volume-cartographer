cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(reslicer)

find_package(OpenCV REQUIRED)

add_executable(reslicer reslicer.cpp)

target_link_libraries(reslicer ${OpenCV_LIBS})
