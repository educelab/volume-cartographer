cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
SET(CMAKE_CXX_FLAGS "-std=c++11")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

add_subdirectory(./../volumepkg volumepkg)
include_directories(${VOLUMEPKG_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${O})
add_definitions(${PCL_DEFINITIONS})

add_executable(sliceIntersection main.cpp CMesh.cpp CPlyHelper.cpp sliceIntersection.cpp)

target_link_libraries(sliceIntersection ${PCL_LIBRARIES})
target_link_libraries(sliceIntersection volumepkg ${OpenCV_LIBS})
