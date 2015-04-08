cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
SET(CMAKE_CXX_FLAGS "-std=c++11")

add_executable(vc_reslicer reslicer.cpp)

target_link_libraries(vc_reslicer volumepkg ${OpenCV_LIBS} ${PCL_LIBRARIES})
