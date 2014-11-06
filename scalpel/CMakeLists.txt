cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
SET(CMAKE_CXX_FLAGS "-std=c++11")

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)


include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(vc_scalpel scalpel.cpp)

target_link_libraries(vc_scalpel ${PCL_LIBRARIES})
target_link_libraries(vc_scalpel ${OpenCV_LIBS})
