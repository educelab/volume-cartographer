cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
SET(CMAKE_CXX_FLAGS "-std=c++11")

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)

add_subdirectory(./../volumepkg volumepkg)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${VOLUMEPKG_INCLUDE_DIR})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(scalpel scalpel.cpp)

target_link_libraries(scalpel ${PCL_LIBRARIES})
target_link_libraries(scalpel ${OpenCV_LIBS})
target_link_libraries(scalpel volumepkg)
