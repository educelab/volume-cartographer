cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
SET(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV REQUIRED)

add_executable(pathgen pathGen.cpp)

target_link_libraries(pathgen ${OpenCV_LIBS})