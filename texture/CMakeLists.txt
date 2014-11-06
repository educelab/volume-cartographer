cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
SET(CMAKE_CXX_FLAGS "-std=c++11")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(OpenNI)
#find_package(OpenNI2)

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${O})
add_definitions(${PCL_DEFINITIONS})

add_executable(vc_texture CMesh.cpp CPlyHelper.cpp sliceIntersection.cpp)

target_link_libraries(vc_texture ${PCL_LIBRARIES})
target_link_libraries(vc_texture ${OpenCV_LIBS})
