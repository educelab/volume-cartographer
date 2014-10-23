cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
SET(CMAKE_CXX_FLAGS "-std=c++11")

add_subdirectory(./../volumepkg volumepkg)
include_directories(${VOLUMEPKG_INCLUDE_DIR})

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(reslicer reslicer.cpp)

target_link_libraries(reslicer volumepkg ${OpenCV_LIBS})
